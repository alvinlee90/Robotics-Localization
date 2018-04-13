#include "particle_filter.hpp"

ParticleFilter::ParticleFilter(ros::NodeHandle node)
    : n_(node)
{
    // Initialize shit
    InitializeParameters();
    InitializeParticles();
    InitializeLaser();
    InitializeProcessNoise();
    InitializeMeasurementModel(); 
    InitializeMap();

    // Set subscriber and publishers
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ParticleFilter::ScanCallback, this);
    odometry_sub_ = n_.subscribe("/robot/wheel_vel", 1, &ParticleFilter::OdometryCallback, this);

    pose_pub_ = n_.advertise<geometry_msgs::Pose2D>("/dead_reckoning/particle_filter", 1);
    particle_pub_ = n_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);

    // Time for odometry 
    previous_time = ros::Time::now();

    // Initialize wheel velocities
    w_left = 0.0;
    w_right = 0.0;

    #ifdef DEBUG
    // ROS_INFO("[PF::Initialize] Initial Pose");
    // particles.rows(0,2).print(); 
    #endif
}

void ParticleFilter::InitializeParameters()
{
    // Initialize the parameters of the particle filter

    // Measurement parameters
    if (!n_.getParam("/localisation/robot/base", robot_base) ||
        !n_.getParam("/localisation/robot/radius", wheel_radius))
        ROS_ERROR("[PF::init] Failed to load number of measurements");

    // Number of measurements parameters
    if (!n_.getParam("/localisation/num_measurements", num_measurements) ||
        !n_.getParam("/localisation/max_measurements", max_measurements))
        ROS_ERROR("[PF::init] Failed to load number of measurements");

    default_measurements = num_measurements; 
    
    // Update parameters
    if (!n_.getParam("/localisation/update_rate", update_rate))
        ROS_ERROR("[PF::init] Failed to update rate");
    
    // Outlier threshold
    if (!n_.getParam("/localisation/outlier/outlier_threshold", outlier_threshold) ||
        !n_.getParam("/localisation/outlier/max_outlier", max_outlier))
        ROS_ERROR("[PF::init] Failed to update outlier parameters");
}

void ParticleFilter::InitializeProcessNoise() 
{
    // Initialize the process noise models 

    // Set processprocess noise [std_x, std_y, std_theta]'
    process_noise_std.set_size(3);
    if (!n_.getParam("/localisation/process_var/position", process_noise_std(0)) ||
        !n_.getParam("/localisation/process_var/orientation", process_noise_std(2)))
    {
        ROS_ERROR("[PF::init] Failed to load process noise variance noise.");
    }

    // Set standard diviation for noise of x and y as the same
    process_noise_std(1) = process_noise_std(0); 
}

void ParticleFilter::InitializeMeasurementModel()
{
    //  Function to initialize the measurement noise models

    double m_noise_std;
    // Get measurement model 
    if (!n_.getParam("/localisation/measurement_model/measurement_std", m_noise_std))
        ROS_ERROR("[PF::init] Failed to load process measurement variance noise.");

    // Constants for gaussian probability calculations
    gaus_exp_const = -1.0 / (2 * std::pow(m_noise_std, 2));
    gaus_prob_const = 1.0 / (m_noise_std * std::sqrt(2 * M_PI));
}

void ParticleFilter::InitializeParticles() 
{
    // Function to initialize particles

    // Number of particles
    if (!n_.getParam("/localisation/num_particles", num_particles))
    {
        ROS_ERROR("[PF::init] Failed to load number of particles.");
    }

    // Vector for systematic-resampling
    resample_consts.set_size(num_particles); 
    resample_consts.fill(1.0 / (double)num_particles);
    resample_consts(0) = 0.0; 
    resample_consts = arma::cumsum(resample_consts); 

    // Initial pose and position of the particle set [x, y, theta]'
    arma::vec initial_odometry(3);
    if (!n_.getParam("/localisation/initial_pose/x", initial_odometry(0)) ||
        !n_.getParam("/localisation/initial_pose/y", initial_odometry(1)) ||
        !n_.getParam("/localisation/initial_pose/theta", initial_odometry(2)))
    {
        ROS_ERROR("[PF::init] Failed to load initial pose of particles.");
    }

    // Initialize particle set with initial pose and even weights
    particles.set_size(4, num_particles);
    particles.rows(0, 2).each_col() = initial_odometry;
    particles.row(3).fill(1.0 / (double)num_particles); 

    // Initial noise
    arma::vec noise_std(3);
    if (!n_.getParam("/localisation/initial_pose/position_std", noise_std(0)) ||
        !n_.getParam("/localisation/initial_pose/orientation_std", noise_std(2)))
    {
        ROS_ERROR("[PF::init] Failed to load noise of the initial pose of particles.");
    }
    // x and y noise parameters are the same
    noise_std(1) = noise_std(0); 

    // Add initial uncertainty/noise to the particle set
    arma::mat process_noise(3, particles.n_cols, arma::fill::randn);
    particles.rows(0, 2) += process_noise % arma::repmat(noise_std, 1, particles.n_cols);
}

void ParticleFilter::InitializeLaser()
{
    // Initialize laser parameters 

    // Set laser scan parameters
    int num_scan; 
    if (!n_.getParam("/localisation/laser/num_scan", num_scan))
    {
        ROS_ERROR("[PF::init] Failed to load laser scan parameters.");
    }

    // Generate vector of laser scan cell index numbers
    scan_idx = arma::linspace<arma::uvec>(0, num_scan - 1, num_scan);

    // Randomly shuffle the index numbers
    scan_idx = arma::shuffle(scan_idx);

    // Set laser angle parameters
    double angle_dir;    
    if (!n_.getParam("/localisation/laser/angle_min", laser.angle_min) ||
        !n_.getParam("/localisation/laser/angle_dir", angle_dir))
    {
        ROS_ERROR("[PF::init] Failed to load laser angle parameters.");
    }
    // Theta values in the parameter server are multiplicative
    // constants of PI [in rad units]
    laser.angle_min *= M_PI;
    laser.angle_inc = angle_dir * 2 * M_PI / (double)num_scan;

    // Set laser range parameters
    if (!n_.getParam("/localisation/laser/range_min", laser.range_min) ||
        !n_.getParam("/localisation/laser/range_max", laser.range_max))
    {
        ROS_ERROR("[PF::init] Failed to load laser range parameters.");
    }

    // Set laser scan transform
    double tf_theta; 
    laser.tf_position.set_size(2,1);
    if (!n_.getParam("/localisation/laser/tf_x", laser.tf_position(0)) ||
        !n_.getParam("/localisation/laser/tf_y", laser.tf_position(1)) ||
        !n_.getParam("/localisation/laser/tf_theta", tf_theta))
    {
        ROS_ERROR("[PF::init] Failed to load laser tf parameters.");
    }
    // Theta values in the parameter server are multiplicative
    // constants of PI [in rad units]
    laser.angle_min += tf_theta * M_PI; 

    // Transform the particle positions
    particles.rows(0, 1).each_col() += laser.tf_position;

    #ifdef DEBUG
    ROS_INFO("[PF::LaserInit] Min: %f, Inc: %f", laser.angle_min, laser.angle_inc);
    // map_lines1.print(); 
    // map_lines2.print();
    // map_lines.each_slice([](arma::mat &x) { x.print(); });
    #endif
}

void ParticleFilter::InitializeMap()
{
    // Function to initial the map parameters
    std::string filename, line;
    std::ifstream map_fs;

    // Set laser scan parameters
    if (!n_.getParam("/localisation/map/filename", filename))
    {
        ROS_ERROR("[PF::init] Failed to load map path.");
    }

    // Open map file 
    map_fs.open((filename).c_str());

    if (map_fs.fail())
    {
        ROS_ERROR("[PF::init] Failed to open map file: %s", filename.c_str());
        return; 
    }

    arma::mat map_lines1, map_lines2;
    arma::vec tmp_vec1(2), tmp_vec2(2);

    while (getline(map_fs, line))
    {
        if (line[0] == '#') { continue; }       // Commented line

        // Save contents in line (format: {x1} {y1} {x2} {y2})
        std::istringstream line_stream(line);

        // Save to temp vector (format: {x1} {y1} {x2} {y2})
        line_stream >> tmp_vec1(0) >> tmp_vec1(1) >> tmp_vec2(0)  >> tmp_vec2(1);

        // Append vector to end of map matrix
        map_lines1.insert_cols(map_lines1.n_cols, tmp_vec1);
        map_lines2.insert_cols(map_lines2.n_cols, tmp_vec2);
    }

    map_lines = arma::join_slices(map_lines1, map_lines2);

    #ifdef DEBUG
    ROS_INFO("Map lines before transform");
    map_lines.print();
    #endif

    double theta; 
    // Map offset parameters (offset between /map and /odometry frame)
    if (!n_.getParam("/localisation/map/x_offset", tmp_vec1(0)) ||
        !n_.getParam("/localisation/map/y_offset", tmp_vec1(1)) ||
        !n_.getParam("/localisation/map/theta_offset", theta))
    {
        ROS_ERROR("[PF::init] Failed to load map offsets.");
    }

    // Translate map coordinates
    map_lines.slice(0).each_col() += tmp_vec1;
    map_lines.slice(1).each_col() += tmp_vec1;

    // Transform map to odometry frame [/map -> /odometry frame]
    // Theta values in the parameter server are multiplicative 
    // constants of PI [in rad units]
    theta *= M_PI;

    // Rotate map coordinates
    rotation_mat_map = {{std::cos(theta), -std::sin(theta)},
                        {std::sin(theta), std::cos(theta)}};
    map_lines = rotation_mat_map * map_lines.each_slice();

    // Max and min coordinate values for the particles coordinates
    min_coord_values = arma::min(arma::min(map_lines, 1), 2);
    max_coord_values = arma::max(arma::max(map_lines, 1), 2);

    #ifdef DEBUG
    ROS_INFO("Map lines after transform");
    map_lines.print();
    ROS_INFO("[PF::Map] Min coordinate values");
    min_coord_values.print();
    ROS_INFO("[PF::Map] Max coordinate values");
    max_coord_values.print();
    #endif
}

void ParticleFilter::OdometryCallback(const wheel_sensor::WheelVelocity &msg)
{
    // Function for the odometry call back

    // Store wheel velocities
    w_left = (double)msg.left;
    w_right = (double)msg.right;

    #ifdef DEBUG
    // ROS_INFO("[PF::Odometry] Measurement");
    // odometry_now.print();
    #endif 
}

void ParticleFilter::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // Function for the scan call back; 
    // Read the scan and obtain X number of valid scan readings

    #ifdef DEBUG
    // ros::Time begin = ros::Time::now();
    #endif

    // Matrix for scan readings
    scan_data.set_size(2, num_measurements);

    // Obtain # of scan measurements for update step
    arma::uword j = 0;
    for (arma::uword i = 0; i < scan_data.n_cols; ++i)
    {
        // Get next valid reading
        while ((msg->ranges[scan_idx(j)] < laser.range_min) || 
               (msg->ranges[scan_idx(j)] > laser.range_max))
            j++;    // Not a valid scan measurement

        // Store valid scan measurement and angle in matrix
        scan_data(0, i) = msg->ranges[scan_idx(j)];
        scan_data(1, i) = std::remainder(laser.angle_min + (scan_idx(j) * laser.angle_inc), 2 * M_PI);

        j++;

        // // Error check
        // if (j >= scan_idx.n_elem)
        //     j = 0;
    }

    SetInputScan();

    #ifdef DEBUG
    // ros::Duration duration = ros::Time::now() - begin;
    // ROS_INFO("[Scan Callback] Time: %f", duration.toSec());
    #endif
}

void ParticleFilter::Predict()
{
    // Function for the predict phase

    // Calculate the delta time
    ros::Time time_now = ros::Time::now();
    double delta_time = (time_now - previous_time).toSec();
    previous_time = time_now;

    // Calculate change in pose
    double delta_d = (wheel_radius * (w_right + w_left) / 2) * delta_time;
    double delta_theta = wheel_radius * ((w_right - w_left) / robot_base) * delta_time;

    // Calculate the predicted pose
    particles.row(0) += delta_d * arma::cos(particles.row(2));
    particles.row(1) += delta_d * arma::sin(particles.row(2));
    particles.row(2) += delta_theta;  

    // Add process noise
    arma::mat process_noise(3, num_particles, arma::fill::randn);
    particles.rows(0, 2) += process_noise.each_col() % process_noise_std;

    // Keep pose within the map
    particles.rows(0, 1) = arma::max(arma::min(particles.rows(0, 1), 
                                     arma::repmat(max_coord_values, 1, particles.n_cols)),
                                     arma::repmat(min_coord_values, 1, particles.n_cols));

    IncrementPredictCount();

    #ifdef DEBUG
    // ROS_INFO("[PF::Predict] Predict process value");
    // process_predict.print(); 
    // ROS_INFO("[PF::Predict] Particle pose after predict");
    // particles.print();
    #endif
}

void ParticleFilter::Update()
{
    // Function for the update phase  

    #ifdef DEBUG
    // ros::Time begin = ros::Time::now();
    // ROS_INFO("[PF::Update] Before theta normalisation"); 
    // particles.print(); 
    #endif

    // Normalize theta between [-pi, pi)
    particles.row(2).transform([](double x) { return (std::remainder(x, 2 * M_PI)); });

    #ifdef DEBUG
    // ROS_INFO("[PF::Update] After theta normalisation"); 
    // particles.print(); 
    #endif

    arma::mat range(scan_data.n_cols, particles.n_cols);

    for (arma::uword i = 0; i < particles.n_cols; ++i)
    {
        // Find ray intersections for all particles
        range.col(i) = RayCasting(particles.col(i));
    }

    // Update weights
    UpdateWeights(range);

    #ifdef DEBUG
    // ROS_INFO("[PF::Update] Ray casting distances");
    // range.t().print(); 
    // ROS_INFO("[PF::Update] Scan data distances");
    // scan_data.print();
    // ROS_INFO("[PF::weights] Updated weights.");
    // particles.row(3).print();
    #endif

    // Resample particles
    SystematicResampling(); 

    #ifdef DEBUG
    // ros::Duration duration = ros::Time::now() - begin; 
    // ROS_INFO("[PF::Update] Time = %f", duration.toSec());
    #endif
}

arma::vec ParticleFilter::RayCasting(arma::mat input_particle)
{   
    // Function to calculate the range/distance of the laser
    // scan reading for each particle

    // Vector of the range distances for particle 
    arma::vec range_dist(scan_data.n_cols);

    // Translate map to set particle to [0,0]
    arma::cube tf_map_lines(arma::size(map_lines));

    tf_map_lines.slice(0) = map_lines.slice(0).each_col() - input_particle.submat(0, 0, 1, 0);
    tf_map_lines.slice(1) = map_lines.slice(1).each_col() - input_particle.submat(0, 0, 1, 0);
    
    for (arma::uword i = 0; i < scan_data.n_cols; ++i)
    {
        arma::cube scan_map_lines;
        arma::uvec line_idx;

        double theta = -(input_particle(2, 0) + scan_data(1, i));
        do
        {
            // Create rotation matrix for the map (odometry + laser_scan angle)
            // Translate the map (odometry + laser offset)
            double sin_theta = std::sin(theta);
            double cos_theta = std::cos(theta);
            arma::mat rotation_mat = {{cos_theta, -sin_theta},
                                      {sin_theta, cos_theta}};

            // Rotate the map
            scan_map_lines = rotation_mat * tf_map_lines.each_slice();

            // Check for ray intersections
            // Map lines format:
            // slice_(0) = point_1; slice(1) = point_2; row(0) = x; row(1) = y
            // Conditions for intersection:
            //  - either x1 and x2 must be positive     [x-axis intersection]
            //  - y1 and y2 must have different signs   [x-axis intersection]
            arma::umat ray_check = ((scan_map_lines.slice(0).row(0) > 0) ||
                                    (scan_map_lines.slice(1).row(0) > 0)) &&
                                   ((scan_map_lines.slice(0).row(1) >= 0) !=
                                    (scan_map_lines.slice(1).row(1) > 0));

            // Find index of line that intersects with ray
            line_idx = arma::find(ray_check == 1);

            // Increment theta by 0.5 degree incase scan passes through corner of maze
            theta += 0.001; 
        } while (line_idx.is_empty() == true);

        // Map lines format:
        // slice_(0) = point_1; slice(1) = point_2; row(0) = x; row(1) = y
        // Calculate the range distance to all map lines that intersect [x-axis intersection]
        arma::vec range = (scan_map_lines.slice(0).elem(idx2ele(line_idx, 0)) %
                           scan_map_lines.slice(1).elem(idx2ele(line_idx, 1)) - 
                           scan_map_lines.slice(1).elem(idx2ele(line_idx, 0)) %
                           scan_map_lines.slice(0).elem(idx2ele(line_idx, 1))) / 
                          (scan_map_lines.slice(1).elem(idx2ele(line_idx, 1)) - 
                           scan_map_lines.slice(0).elem(idx2ele(line_idx, 1)));

        arma::uvec range_idx = arma::find(range > 0);
        if (range_idx.is_empty() != true)
        {
            // Filter negative distances
            range = range.elem(range_idx);
            range_dist(i) = arma::min(range);
        }
        else
        {
            range_dist(i) = arma::max(range);
        }
    }

    // Return a vector of distances calculated from ray casting
    return range_dist; 
}

void ParticleFilter::UpdateWeights(arma::mat range)
{   
    // Function to update the weights based on the measurements/innovation

    // Calculate the probability from a Gaussian distribution
    arma::mat prob = arma::exp((arma::square(range.each_col() - scan_data.row(0).t())) * gaus_exp_const) * gaus_prob_const;

    // Check for outliers (mean prob of all particles less than a threshold)
    arma::uvec outliers = arma::find(arma::mean(prob, 1) < outlier_threshold);

    if (outliers.is_empty() != true)
    {
        // Randomly shuffle the index numbers
        scan_idx = arma::shuffle(scan_idx);

        // Increase the number of measurement readings to take
        num_measurements = max_measurements; 
        
        #ifdef DEBUG
		ROS_INFO("Number of outliers: %d", (int)outliers.size()); 
		arma::mean(prob, 1).print();
		#endif

        if ((int)outliers.size() > max_outlier)
        {
            particles.row(3).fill(1.0 / (double)particles.n_cols);
            return; 
        }

        // Remove outliers (set prob to 1)
        arma::rowvec ones_row(prob.n_cols, arma::fill::ones);
        prob.each_row(outliers) = ones_row;
    }
    else
    {
        // Reset the number of measurement to the default
        num_measurements = default_measurements;
    }
    
    // Update weights
    arma::rowvec weight = arma::prod(prob, 0);  
    // if (arma::sum(weight) == 0)
    // {
    //     particles.row(3).fill(1.0 / (double)particles.n_cols);
    //     return; 
    // }
        
    particles.row(3) = arma::normalise(weight);    
}

void ParticleFilter::SystematicResampling()
{
    // Function for systematic resampling during the update phase

    // Cumulative probability distribution for resampling
    arma::rowvec weight_cumsum = arma::cumsum(particles.row(3));

    // Random numbers (systematic re-sampling)
    std::default_random_engine rng;
    std::uniform_real_distribution<> rand_sample(0.0, 1.0 / num_particles);
    arma::rowvec random_num = resample_consts + rand_sample(rng);

    // Resample the particles
    arma::mat tmp_particles(4, num_particles);
    int j = 0;

    for (int i = 0; i < num_particles; ++i)
    {
        while (random_num(i) > weight_cumsum(j)) { j++;}
        tmp_particles.col(i) = particles.col(j);
    }

    // Update the particles
    particles = tmp_particles;

    #ifdef DEBUG
    // ROS_INFO("[PF::Resample] Random numbers generated");
    // random_num.print();
    // ROS_INFO("[PF::Resample] New Particles");
    // particles.rows(0,2).print(); 
    #endif
}

void ParticleFilter::PublishParticles()
{
    // Function to publish all the particles in the set to Rviz

    geometry_msgs::PoseArray particles_msg;
    particles_msg.header.stamp = ros::Time::now();
    particles_msg.header.frame_id = "odom";

    // Loop through all the particles
    for (int j = 0; j < num_particles; ++j)
    {
        // Publish individual Pose message for each particle
        geometry_msgs::Pose pose;
        pose.position.x = particles(0, j);
        pose.position.y = particles(1, j);
        pose.orientation = tf::createQuaternionMsgFromYaw(particles(2, j));

        // Add to PoseArray message
        particles_msg.poses.insert(particles_msg.poses.begin(), pose);
    }
    particle_pub_.publish(particles_msg);
}

void ParticleFilter::PublishPose()
{
    // // Function to publish the pose to ROS topic

    // // Pose message
    // geometry_msgs::PoseStamped pose_msg;

    // // Pose is calculated by the average of all particles
    // arma::vec pose_mean(3);
    // pose_mean.rows(0, 1) = arma::mean(particles.rows(0, 1), 1);
    // pose_mean(2) = AverageAngles(particles.row(2));

    // // Publish message
    // pose_msg.pose.position.x = pose_mean(0);
    // pose_msg.pose.position.y = pose_mean(1);
    // pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose_mean(2));
    // pose_msg.header.frame_id = "/odom";
    // pose_msg.header.stamp = ros::Time::now();

    // pose_pub_.publish(pose_msg);

    // #ifdef DEBUG
    // // ROS_INFO("[PF::Pose] Particles pose");
    // // particles.print(); 
    // // ROS_INFO("[PF::Pose] Average pose");
    // // pose_mean.print(); 
    // #endif

    geometry_msgs::Pose2D pose_msg; 
    // Pose is calculated by the average of all particles
    arma::vec pose_mean(3);
    pose_mean.rows(0, 1) = arma::mean(particles.rows(0, 1), 1);
    pose_mean(2) = AverageAngles(particles.row(2));

    // Publish message
    pose_msg.x = pose_mean(0);
    pose_msg.y = pose_mean(1);
    pose_msg.theta = pose_mean(2);

    pose_pub_.publish(pose_msg);
}

double ParticleFilter::AverageAngles(arma::rowvec theta)
{
    // Function to obtain the average of a set of angles [rad]

    // Convert theta to unit circle x/y coordinates
    arma::rowvec x = arma::cos(theta);
    arma::rowvec y = arma::sin(theta);

    // Take the average and atan back to angle [rad]
    return std::atan2(arma::mean(y), arma::mean(x));
}

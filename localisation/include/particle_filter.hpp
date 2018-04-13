#ifndef _PARTICLE_FILTER_
#define _PARTICLE_FILTER_

#define DEBUG

#include <ros/ros.h>
#include <ros/time.h>

// Messages
#include <sensor_msgs/LaserScan.h> //scan

// Odometry
#include <geometry_msgs/PoseStamped.h>
#include <wheel_sensor/WheelVelocity.h>

// Pose
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>

// std includes
#include <math.h>
#include <iostream>
#include <algorithm>

// Transform
#include <tf/tf.h> 

// Test
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

// Matrix library
#include <armadillo>

struct LaserModel
{
    // Laser parameters
    double angle_min, angle_inc;
    double range_min, range_max;
    
    // Transform from /laser to /odometry frame
    arma::mat tf_position;      // [x, y]'
};

class ParticleFilter 
{
public:
    ParticleFilter(ros::NodeHandle node);
    virtual ~ParticleFilter() {}

    // Initialize functions
    void InitializeParameters();
    void InitializeParticles();
    void InitializeProcessNoise(); 
    void InitializeLaser();
    void InitializeMap();
    void InitializeMeasurementModel(); 

    // Predict functions
    void Predict();

    // Update functions
    void Update();
    arma::vec RayCasting(arma::mat particle);
    void UpdateWeights(arma::mat range);
    void SystematicResampling(); 

    // Rviz functions
    void PublishParticles();

    // Publish functions
    void PublishPose();
    double AverageAngles(arma::rowvec theta);

    // Function to convert arma::vec index # to arma::mat element # (ray casting)
    inline arma::uvec idx2ele(arma::uvec col, int row)
    {
        return col * 2 + row;
    }

    // Control functions
    bool GetUpdate() { return (input_scan && (predict_count >= update_rate)); }
    void ResetPredict() { predict_count = 0; }
    void ResetScan() { input_scan = false;}
    inline void IncrementPredictCount() { predict_count++; }
    inline void SetInputScan() { input_scan = true; }

private: 
    ros::NodeHandle n_;

    // Robot attributes
    double robot_base, wheel_radius;
    double w_left, w_right;
    ros::Time previous_time;

    // Particle filter parameters
    int num_particles;
    int num_measurements, default_measurements, max_measurements;

    // Outlier parameters
    int max_outlier;
    double outlier_threshold;

    // Process attributes
    arma::vec process_noise_std;                    // Noise std_dev [x, y, theta]'
    arma::vec min_coord_values, max_coord_values;   // Max/min  map coordinates [x, y]'

    // Measurement attributes
    double gaus_exp_const, gaus_prob_const;

    // Update attributes
    arma::rowvec resample_consts;

    // Map attributes
    // arma::mat map_lines1, map_lines2;
    arma::cube map_lines; 
    arma::mat rotation_mat_map; 

    // Laser attributes
    LaserModel laser;
    arma::uvec scan_idx;
    arma::mat scan_data; 

    // Particle (pose and weight) attribute
    arma::mat particles; 

    // Control attributes
    bool input_scan = false;
    int predict_count = 0; 
    int update_rate;

    // Subscriber attributes and callbacks
    ros::Subscriber scan_sub_, odometry_sub_;
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void OdometryCallback(const wheel_sensor::WheelVelocity &msg);

    // Publisher attributes
    ros::Publisher particle_pub_, pose_pub_; 
};

#endif

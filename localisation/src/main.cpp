#include "particle_filter.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");

    ros::NodeHandle nh;

    ros::Duration(2.0).sleep(); 
    
    ParticleFilter particle_filter(nh);

    particle_filter.PublishParticles();

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        // ros::Time begin = ros::Time::now();

        ros::spinOnce();

        // Predict phase
        particle_filter.Predict();

        // Delayed updated phase 
        if (particle_filter.GetUpdate())
        {
            particle_filter.Update();
            particle_filter.ResetPredict();
        }

        // Publish particles
        // particle_filter.PublishParticles();
        particle_filter.PublishPose();

        // Reset control variables
        particle_filter.ResetScan();
        loop_rate.sleep();

        // ros::Duration duration = ros::Time::now() - begin;
        // ROS_INFO("[PF::Update] Time = %f", duration.toSec());
    }

    return 0;
}

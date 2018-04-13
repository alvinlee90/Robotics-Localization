#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <wheel_sensor/WheelVelocity.h>

#include <iostream>
#include <cmath>

#include <tf/tf.h> //Quanternion tf
#include <tf/transform_broadcaster.h>

class DeadReckoning {
public:
    DeadReckoning(ros::NodeHandle node);
    virtual ~DeadReckoning() {}

    void publishPose();

private:
    ros::NodeHandle n_;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher pose_pub;

    ros::Subscriber sub_motor, particle_filter;

    void motorCallback(const wheel_sensor::WheelVelocity &input);
    void particleFilterCallback(const geometry_msgs::Pose2D &input); 
    
    // Robot parameters
    double radius;
    double base;

    double w_left, w_right;
    double x, y, theta; // intial values to zero.
   
	geometry_msgs::PoseStamped pose;

    ros::Time previous_time_; 

    geometry_msgs::TransformStamped odom_trans;
};

DeadReckoning::DeadReckoning(ros::NodeHandle node):
    n_(node)
{
    previous_time_ = ros::Time::now(); 

    n_.getParam("/dead_reckoning/base", base);
    n_.getParam("/dead_reckoning/radius", radius);
    n_.getParam("/dead_reckoning/x",x);
    n_.getParam("/dead_reckoning/y",y);
    if(!n_.getParam("/dead_reckoning/theta",theta)){
        ROS_ERROR("Failed to load theta value.");
    }
    
    pose_pub = n_.advertise<geometry_msgs::PoseStamped>("/dead_reckoning/PoseStamped", 1);

    sub_motor = n_.subscribe("/robot/wheel_vel", 1, &DeadReckoning::motorCallback, this);
    particle_filter = n_.subscribe("/dead_reckoning/particle_filter", 1, &DeadReckoning::particleFilterCallback, this);

    w_left = 0.0;
    w_right = 0.0;
}

void DeadReckoning::motorCallback(const wheel_sensor::WheelVelocity &input)
{
    w_left = (double)input.left;
    w_right = (double)input.right;
    ROS_INFO("[Speed] Left: %.2lf Right: %.2lf", w_left, w_right);
}

void DeadReckoning::particleFilterCallback(const geometry_msgs::Pose2D &input)
{
    x = (double)input.x;
    y = (double)input.y;
    theta = (double)input.theta; 
}

void DeadReckoning::publishPose()
{
    // Calculate the delta time
    ros::Time time_now = ros::Time::now(); 
    double delta_time = (time_now - previous_time_).toSec(); 
    previous_time_ = time_now; 

    // Calculate change in pose
    double delta_d = (radius * (w_right + w_left) / 2) * delta_time;
    double delta_theta = radius * ((w_right - w_left) / base) * delta_time; 

    // Update pose
    x += delta_d * std::cos(theta); 
    y += delta_d * std::sin(theta); 
    theta = fmod((theta + delta_theta + M_PI),(2*M_PI)) - M_PI; // Normalize theta (-180 180)

    // Create time PoseStamped 
    pose.header.stamp = time_now;
    pose.header.frame_id = "/odom";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    // Broadcaster
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "robot";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = pose.pose.orientation;
    odom_broadcaster.sendTransform(odom_trans);

    // Publish message to publisher 
    pose_pub.publish(pose);
    
    ROS_INFO("[Check] delta_d = %lf\tdelta_theta = %lf\n", delta_d, delta_theta); 
    ROS_INFO("[Odometry] x: %f\ty: %f\ttheta: %f", x, y, (theta*180/M_PI));
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "dead_reckoning");

    ros::NodeHandle n;
    ros::Duration(1.0).sleep(); 
    
    DeadReckoning dead_reckoning(n);

    ros::Rate loop_rate(20);    // loop rate

    while (ros::ok()) {
        ros::spinOnce();
        dead_reckoning.publishPose();
        loop_rate.sleep();
    }
}

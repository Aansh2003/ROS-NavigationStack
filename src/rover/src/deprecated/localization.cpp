#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include "std_srvs/Trigger.h"
#include "std_msgs/Empty.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <math.h>

class localization 
{
    public:
        sensor_msgs::Imu finalMsg;
        ros::Publisher fixed_pub;
        localization(ros::NodeHandle n)
        {
            fixed_pub = n.advertise<sensor_msgs::Imu>("/zed2/filtered/imu", 1);
        }
        void robotLocCallback (const sensor_msgs::Imu::ConstPtr &odom);
    
};

void localization::robotLocCallback (const sensor_msgs::Imu::ConstPtr &imu)
{
    finalMsg.header = imu->header;
    finalMsg.angular_velocity = imu->angular_velocity;
    finalMsg.linear_acceleration = imu->linear_acceleration;
    finalMsg.orientation = imu->orientation;
    finalMsg.angular_velocity_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
    finalMsg.linear_acceleration_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
    finalMsg.orientation_covariance = {0.00000, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.00000};
    fixed_pub.publish(finalMsg);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "localizationNode");
    ros::NodeHandle n;
    ros::Rate rate(60);

    // Subscribers 
    localization Localization(n);
    
    ros::Subscriber robotLocalization_sub = n.subscribe("/zed2/imu/data", 1, &localization::robotLocCallback, &Localization); 
    rate.sleep();
    ros::spin();
}
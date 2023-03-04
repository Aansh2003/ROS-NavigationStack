#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Trigger.h"

class remap
{
public:
    ros::Publisher zed2Odom_pub;
    ros::Publisher zed2Imu_pub;
    ros::Publisher wheelOdom_pub;

    remap(ros::NodeHandle n)
    {
        zed2Imu_pub = n.advertise<sensor_msgs::Imu>("/zed2/imu_data/fixed", 10);
        wheelOdom_pub = n.advertise<nav_msgs::Odometry>("/wheel_odom/fixed", 10);
        zed2Odom_pub = n.advertise<nav_msgs::Odometry>("/zed2/odom/fixed", 10);
    }

    void zed2Odom_callback(const nav_msgs::Odometry::ConstPtr &zed2Odom_msg);
    void wheelOdom_callback(const nav_msgs::Odometry::ConstPtr &wheelOdom_msg);
    void zed2Imu_callback(const sensor_msgs::Imu::ConstPtr &zed2Imu_msg);

private:
    sensor_msgs::Imu zed2Imu_fixed;
    nav_msgs::Odometry wheelOdom_fixed;
    nav_msgs::Odometry zed2Odom_fixed;
};

void remap::zed2Odom_callback(const nav_msgs::Odometry::ConstPtr &zed2Odom_msg)
{
    zed2Odom_fixed.child_frame_id = zed2Odom_msg->child_frame_id;
    zed2Odom_fixed.header = zed2Odom_msg->header;
    zed2Odom_fixed.pose.pose.position= zed2Odom_msg->pose.pose.position;
    zed2Odom_fixed.pose.covariance = {0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0};
    zed2Odom_fixed.twist.twist = zed2Odom_msg->twist.twist;
    zed2Odom_fixed.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    zed2Odom_pub.publish(zed2Odom_fixed);
}
void remap::wheelOdom_callback(const nav_msgs::Odometry::ConstPtr &wheelOdom_msg)
{
    wheelOdom_fixed.child_frame_id = wheelOdom_msg->child_frame_id;
    wheelOdom_fixed.header = wheelOdom_msg->header;
    wheelOdom_fixed.pose.pose = wheelOdom_msg->pose.pose;
    wheelOdom_fixed.pose.covariance = {0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0};
    wheelOdom_fixed.twist.twist = wheelOdom_msg->twist.twist;
    wheelOdom_fixed.twist.covariance = {0.0001, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.001};
   wheelOdom_pub.publish(wheelOdom_fixed);
}
void remap::zed2Imu_callback(const sensor_msgs::Imu::ConstPtr &zed2Imu_msg)
{
    zed2Imu_fixed.header = zed2Imu_msg->header;
    zed2Imu_fixed.angular_velocity = zed2Imu_msg->angular_velocity;
    zed2Imu_fixed.linear_acceleration = zed2Imu_msg->linear_acceleration;
    zed2Imu_fixed.orientation = zed2Imu_msg->orientation;
    zed2Imu_fixed.angular_velocity_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
    zed2Imu_fixed.linear_acceleration_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};
    zed2Imu_fixed.orientation_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};

    zed2Imu_pub.publish(zed2Imu_fixed); 
}

void reset(ros::NodeHandle n)
{
    std_msgs::Empty empty_msg;
    std_srvs::Trigger reset_msg;

    ros::Publisher reset_zed2Odom = n.advertise<std_msgs::Empty>("/zed2/reset_odometry", 1);
    ros::ServiceClient reset_wheelOdom = n.serviceClient<std_srvs::Trigger>("/firmware/reset_odometry", 1);

    reset_zed2Odom.publish(empty_msg);
    reset_wheelOdom.call(reset_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "startup");
    ros::NodeHandle n;
    ros::Rate rate(120);

    remap navigation_stack(n);

    ros::Subscriber zed2_odom = n.subscribe("/zed2/odom", 10, &remap::zed2Odom_callback, &navigation_stack);
    ros::Subscriber wheel_odom = n.subscribe("/wheel_odom_with_covariance", 10, &remap::wheelOdom_callback, &navigation_stack);
    ros::Subscriber zed2_imu = n.subscribe("/zed2/imu/data", 10, &remap::zed2Imu_callback, &navigation_stack);

    rate.sleep();

    reset(n);

    ros::spin();
}
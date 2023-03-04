#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TransformStamped.h"
#include "robot_localization/SetPose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>

// Defining Global Variables

// geometry_msgs::PoseWithCovarianceStamped msg;

// class ar_track_transforms
// {
//     private:

//         ros::NodeHandle nh1;
//         ros::NodeHandle nh2;
//         ros::Subscriber sub_arTrack;
//         ros::Publisher pub_arTrack;
//         ros::Publisher pub_transform;
//         ros::Publisher pub_resetZed2Odom;
//         ros::Publisher pub_setPose;
//         ros::ServiceClient pub_resetWheelOdom;

//         int reset_odom = 0;
//         int initial_transform_publisher = 1;

//     public:

//         ar_track_transforms(ros::NodeHandle &ar_track, ros::NodeHandle &transform_pub);

//         void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &ar_msg);

// };

// ar_track_transforms::ar_track_transforms(ros::NodeHandle &ar_track, ros::NodeHandle &transform_pub)
// {
//     nh1 = ar_track;
//     nh2 = transform_pub;

//     sub_arTrack = nh1.subscribe("/ar_pose_marker", 1, &ar_track_transforms::ar_callback, this);

//     pub_setPose = transform_pub.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1);
//     pub_resetZed2Odom = transform_pub.advertise<std_msgs::Empty>("/zed2/reset_odometry", 1);
// }

// void ar_track_transforms::ar_callback (const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &ar_msg)
// {

//     // For publishing transform when AR Tags are in view
//     if (ar_msg->markers[0].header.frame_id == "map")
//     {
//         // publish detected transform

//         msg.pose.pose = ar_msg->markers[0].pose.pose;
//         msg.header = ar_msg->markers[0].header;
//         msg.pose.covariance = {0};

//         // Changing the flag value

//     }

//     // For publishing transform when AR Tags go out of view

// }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "AR_tag_global_pose");
    ros::NodeHandle n;
    ros::Rate rate(30);
    rate.sleep();

    int count = 1;

    while (ros::ok())
    {

        static tf2_ros::TransformBroadcaster br;

        geometry_msgs::TransformStamped transform;

        transform.header.stamp = ros::Time::now();
        transform.header.seq = count;
        count++;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";

        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        br.sendTransform(transform);

        rate.sleep();
    }
}
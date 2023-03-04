#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Trigger.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <math.h>
#include <iostream>
#include <ostream>
#include <map>

struct circle
{
    double x;
    double y;
    double r;
};
struct points
{
    double x1;
    double y1;
    double x2;
    double y2;
};
struct ar_tag
{
    double x;
    double y;
};

class poseDetection
{
public:
    ros::Publisher pose_pub;
    ros::Publisher pose_setter;
    std::map<int, ar_tag> ar_tags;
    static tf2_ros::TransformBroadcaster br;

    poseDetection(ros::NodeHandle n)
    {
        pose_pub = n.advertise<nav_msgs::Odometry>("ar_track/pose", 1);
        pose_setter = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1);

        // Inserting AR tag coordinates on the map
        ar_tags.insert(std::pair<int, ar_tag>(0, {0.00, 0.00}));
        ar_tags.insert(std::pair<int, ar_tag>(1, {10.00, 0.00}));
        ar_tags.insert(std::pair<int, ar_tag>(2, {10.00, 10.00}));
        ar_tags.insert(std::pair<int, ar_tag>(3, {28.35, 0.04}));
        ar_tags.insert(std::pair<int, ar_tag>(4, {21.83, 2.80}));
        ar_tags.insert(std::pair<int, ar_tag>(5, {18.71, 17.19}));
        ar_tags.insert(std::pair<int, ar_tag>(6, {26.95, 7.44}));
        ar_tags.insert(std::pair<int, ar_tag>(7, {15.97, -7.57}));
        ar_tags.insert(std::pair<int, ar_tag>(8, {17.87, 7.57}));
        ar_tags.insert(std::pair<int, ar_tag>(9, {10.00, -10.00}));
        ar_tags.insert(std::pair<int, ar_tag>(10, {29.26, 14.54}));
        ar_tags.insert(std::pair<int, ar_tag>(11, {18.41, 25.83}));
        ar_tags.insert(std::pair<int, ar_tag>(12, {23.34, 14.11}));
        ar_tags.insert(std::pair<int, ar_tag>(13, {8.18, 18.63}));
        ar_tags.insert(std::pair<int, ar_tag>(14, {0.00, 0.00}));
        ar_tags.insert(std::pair<int, ar_tag>(15, {2.27, 16.85}));
    }

    void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &ar_msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

private:
    int count = 0;
    ar_track_alvar_msgs::AlvarMarkers arTrackMsg;
    circle tag1, tag2;
    points final_points, tag_1_dist, tag_2_dist;
    float odom_x, odom_y;
    nav_msgs::Odometry final_odom;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;

    void getARTrackData();
    float getDistance(points a);
    points circlesIntersections(circle a, circle b);
    void posePublisher(points a);
    void transformPublisher();

    geometry_msgs::PoseWithCovarianceStamped pos_set;
};

void poseDetection::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    odom_x = odom_msg->pose.pose.position.x;
    odom_y = odom_msg->pose.pose.position.y;
    pos_set.pose.pose.orientation = odom_msg->pose.pose.orientation;
    
}

void poseDetection::ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &ar_msg)
{
    std::cout << ar_msg->markers.size() << std::endl;
    if (ar_msg->markers.size() >= 2)
    {
        initialFlag =  1;
        tag1.x = ar_tags.at(ar_msg->markers[0].id).x;
        tag1.y = ar_tags.at(ar_msg->markers[0].id).y;
        tag2.x = ar_tags.at(ar_msg->markers[1].id).x;
        tag2.y = ar_tags.at(ar_msg->markers[1].id).y;

        points temp;
        temp.x1 = 0;
        temp.y1 = 0;
        temp.x2 = ar_msg->markers[0].pose.pose.position.x;
        temp.y2 = ar_msg->markers[0].pose.pose.position.y;

        tag1.r = getDistance(temp);

        temp.x1 = 0;
        temp.y1 = 0;
        temp.x2 = ar_msg->markers[1].pose.pose.position.x;
        temp.y2 = ar_msg->markers[1].pose.pose.position.y;

        tag2.r = getDistance(temp);

        final_points = circlesIntersections(tag1, tag2);

        posePublisher(final_points);
    }

    if (initialFlag == 0)
    {

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
    }

    else 
    {

        transform.header.stamp = ros::Time::now();
        transform.header.seq = count;
        count++;
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";

        transform.transform.translation.x = final_odom.pose.pose.position.x;
        transform.transform.translation.y = final_odom.pose.pose.position.y;
        transform.transform.translation.z = 0;

        tf2::Quaternion q;
        transform.transform.rotation.x = orientation_x;
        transform.transform.rotation.y = orientation_y;
        transform.transform.rotation.z = orientation_z;
        transform.transform.rotation.w = orientation_w;

        br.sendTransform(transform);

    }

}

// void poseDetection::transformPublisher()
// {
//     transform.header.stamp = ros::Time::now();
//     transform.header.seq = count;
//     count++;
//     transform.header.frame_id = "map";
//     transform.child_frame_id = "odom";

//     transform.transform.translation.x = 0;
//     transform.transform.translation.y = 0;
//     transform.transform.translation.z = 0;

//     q.setRPY(0, 0, 0);
//     transform.transform.rotation.x = q.x();
//     transform.transform.rotation.y = q.y();
//     transform.transform.rotation.z = q.z();
//     transform.transform.rotation.w = q.w();

//     br.sendTransform(transform);
// }

void poseDetection::posePublisher(points a)
{
    float tag1Dist, tag2Dist;
    tag_1_dist.x1 = odom_x;
    tag_1_dist.y1 = odom_y;
    tag_1_dist.x2 = a.x1;
    tag_1_dist.y2 = a.y1;
    tag_2_dist.x1 = odom_x;
    tag_2_dist.y1 = odom_y;
    tag_2_dist.x2 = a.x2;
    tag_2_dist.y2 = a.y2;
    tag1Dist = getDistance(tag_1_dist);
    tag2Dist = getDistance(tag_2_dist);
    if (tag1Dist > tag2Dist)
    {
        final_odom.pose.pose.position.x = a.x1;
        final_odom.pose.pose.position.y = -a.y1;
        pos_set.pose.pose.position.x = a.x1;
        pos_set.pose.pose.position.y = -a.y1;
        pose_setter.publish(pos_set);
    }
    else
    {
        final_odom.pose.pose.position.x = a.x2;
        final_odom.pose.pose.position.y = -a.y2;
        pos_set.pose.pose.position.x = a.x2;
        pos_set.pose.pose.position.y = -a.y2;
        pose_setter.publish(pos_set);
    }
    pose_pub.publish(final_odom);
    
}

float poseDetection::getDistance(points a)
{
    float distance;
    distance = sqrt(pow((a.x1 - a.x2), 2) + pow((a.y1 - a.y2), 2));

    return distance;
}

points poseDetection::circlesIntersections(circle a, circle b)
{
    points intersection;

    double circle_distance = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
    intersection.x1 = ((a.x + b.x) / 2) +
                      ((pow(a.r, 2) - pow(b.r, 2)) * (b.x - a.x) / (2 * pow(circle_distance, 2))) +
                      (sqrt((2 * (pow(a.r, 2) + pow(b.r, 2)) / pow(circle_distance, 2)) -
                            (pow((pow(a.r, 2) - pow(b.r, 2)), 2) / pow(circle_distance, 4)) - 1) *
                       (b.y - a.y) / 2);
    intersection.x2 = ((a.x + b.x) / 2) +
                      ((pow(a.r, 2) - pow(b.r, 2)) * (b.x - a.x) / (2 * pow(circle_distance, 2))) -
                      (sqrt((2 * (pow(a.r, 2) + pow(b.r, 2)) / pow(circle_distance, 2)) -
                            (pow((pow(a.r, 2) - pow(b.r, 2)), 2) / pow(circle_distance, 4)) - 1) *
                       (b.y - a.y) / 2);
    intersection.y1 = ((a.y + b.y) / 2) +
                      ((pow(a.r, 2) - pow(b.r, 2)) * (b.y - a.y) / (2 * pow(circle_distance, 2))) +
                      (sqrt((2 * (pow(a.r, 2) + pow(b.r, 2)) / pow(circle_distance, 2)) -
                            (pow((pow(a.r, 2) - pow(b.r, 2)), 2) / pow(circle_distance, 4)) - 1) *
                       (a.x - b.x) / 2);
    intersection.y2 = ((a.y + b.y) / 2) +
                      ((pow(a.r, 2) - pow(b.r, 2)) * (b.y - a.y) / (2 * pow(circle_distance, 2))) -
                      (sqrt((2 * (pow(a.r, 2) + pow(b.r, 2)) / pow(circle_distance, 2)) -
                            (pow((pow(a.r, 2) - pow(b.r, 2)), 2) / pow(circle_distance, 4)) - 1) *
                       (a.x - b.x) / 2);

    return intersection;
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
    ros::init(argc, argv, "arTrack_pose");
    ros::NodeHandle n;
    ros::Rate rate(30);

    poseDetection poseObject(n);

    ros::Subscriber sub_artag = n.subscribe("/ar_pose_marker", 1, &poseDetection::ar_callback, &poseObject);
    ros::Subscriber sub_odom = n.subscribe("/odometry/filtered", 1, &poseDetection::odom_callback, &poseObject);

    rate.sleep();

    ros::spin();
}
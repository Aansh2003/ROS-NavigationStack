#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Empty.h>
#include <iostream>
#include <cmath>
class ProbeDeploy{
    public:
        double value_x=0;
        double value_y=0;
        double orientZ=0;
        double orientW=0;
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    private:
        
};
void ProbeDeploy::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    value_x = odom->pose.pose.position.x;
    value_y = odom->pose.pose.position.y;
}
double dist(double x1,double x2,double y1,double y2){
    return std::sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1));
}

int main(int argc,char **argv){

    ProbeDeploy odom;
    double X,Y,distance;
    bool IsScience;
    std_msgs::Empty EMsg;
    geometry_msgs::PoseStamped msg;
    std::cout<<"Enter goal location (X Y):"<<std::endl;
    std::cin>>X>>Y;
    msg.pose.position.x = X;
    msg.pose.position.y = -Y;
    msg.pose.orientation.x=0;
    msg.pose.orientation.y=0;
    msg.pose.orientation.z=1;
    msg.pose.orientation.w=0;
    msg.header.frame_id="map";
    ros::init(argc,argv,"movebase_goal");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    ros::Publisher pub2 = node.advertise<std_msgs::Empty>("/probe_deployment_unit/drop",1);
    ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("/odometry/filtered",10,&ProbeDeploy::OdomCallback,&odom);
    ros::Rate r(10);
    do{
        distance = dist(odom.value_x,msg.pose.position.x,odom.value_y,msg.pose.position.y);
        if(distance<0.2){   
                pub2.publish(EMsg);          
            std::cout<<"Enter next goal location (X Y):"<<std::endl;
            std::cin>>msg.pose.position.x>>msg.pose.position.y;
            msg.pose.position.y = -msg.pose.position.y;
        }
        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
        
    }while(ros::ok());
}

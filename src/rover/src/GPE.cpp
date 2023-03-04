#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class segmentation
{
public:
    ros::Publisher pub;

    // Constructor

    segmentation(std::string sub_topic, std::string pub_topic, ros::NodeHandle n)
    {
        // Initializing Publishers and Subscribers

        pub = n.advertise<sensor_msgs::PointCloud2>(pub_topic, 5);
    }

    // Callback Function

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &PointCloud2);

private:
    // Preprocessing of data
    void preprocessing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing);
    // Ground Plane Elimination
    void ground_plane_elimination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing);
    // Clustering
    void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing);
    sensor_msgs::PointCloud data;
    sensor_msgs::PointCloud2 inter;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_final;
};

void segmentation::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &PointCloud2)
{

    // PCL and PointCloud2 Point cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_pcl2;
    sensor_msgs::PointCloud2 pointcloud2;

    // Converting PointCloud2 to PCL Format

    pcl_conversions::toPCL(*PointCloud2, cloud_pcl2);
    pcl::fromPCLPointCloud2(cloud_pcl2, *cloud);
    preprocessing(cloud);
    ground_plane_elimination(cloud);
    pcl::toPCLPointCloud2(*cloud, cloud_pcl2);
    pcl_conversions::fromPCL(cloud_pcl2, inter);
    inter.header.frame_id = "camera_optical_frame";
    pub.publish(inter);
}

void segmentation::preprocessing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing)
{
    // Pass through filter
    
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_processing);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 3);
    pass.filter(*cloud_processing);

    pcl::PassThrough<pcl::PointXYZRGB> pass2;
    pass2.setInputCloud(cloud_processing);
    pass2.setFilterFieldName("x");
    pass2.setFilterLimits(-1.5,1.5);
    pass2.setFilterLimitsNegative (false);
    pass2.filter(*cloud_processing);


    pcl::PCLPointCloud2::Ptr voxelization(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_processing, *voxelization);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(voxelization);
    sor.setLeafSize(0.04f, 0.04f, 0.04f);
    sor.filter(*voxelization);
    pcl::fromPCLPointCloud2(*voxelization, *cloud_processing);
}

void segmentation::ground_plane_elimination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_processing);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 3.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud_processing);
    reg.setIndices (indices);
    reg.setSearchMethod (tree2);
    reg.setDistanceThreshold (50);
    reg.setPointColorThreshold (10);
    reg.setRegionColorThreshold (10);
    reg.setMinClusterSize (25);
    reg.setMaxClusterSize(100000);

    std::vector <pcl::PointIndices> cluster_indices;
    reg.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& idx : it->indices){
        cloud_cluster->push_back ((*cloud_processing)[idx]); //*
      }
       cloud_cluster->width = cloud_cluster->size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;
       if(cloud_cluster->size ()<500){
         *cloud_f += *cloud_cluster;
       }
     }
     *cloud_processing = *cloud_f;

}


int main(int argc, char **argv)
{
    // Initializing ROS Node

    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle n;
    ros::Rate rate(10);

    std::string sub_topic = "/zed2/point_cloud/cloud_registered";
    std::string pub_topic = "/pcl_cloud";

    segmentation Segmentation(sub_topic, pub_topic, n);

    ros::Subscriber sub = n.subscribe(sub_topic, 1, &segmentation::cloud_callback, &Segmentation);

    rate.sleep();

    // Initiating ROS Spin

    ros::spin();
}

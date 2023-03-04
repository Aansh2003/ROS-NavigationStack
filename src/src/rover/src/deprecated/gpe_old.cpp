#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class segmentation
{
public:
    ros::Publisher pub;

    // Constructor

    segmentation(std::string sub_topic, std::string pub_topic, ros::NodeHandle n)
    {
        // Initializing Publishers and Subscribers

        pub = n.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
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
    pcl_conversions::fromPCL(cloud_pcl2, pointcloud2);

    pub.publish(pointcloud2);
}

void segmentation::preprocessing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_processing)
{
    // Pass through filter

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_processing);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 3.0);
    pass.filter(*cloud_processing);

    // Voxelization

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
    // // Normal Estimation (NOT IN USE CURRENTLY)

    // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    // ne.setInputCloud (cloud_processing);

    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    // ne.setSearchMethod (tree);

    // // Output datasets
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // ne.setRadiusSearch (0.04);

    // // Compute the features
    // ne.compute (*cloud_normals);

    // Plane segmentation using RANSAC

    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    std::vector<int> inliers;

    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud_processing));

    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (.3);
    ransac.computeModel();
    ransac.getInliers(inliers);

    indices->indices = inliers;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_processing);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud_processing);

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

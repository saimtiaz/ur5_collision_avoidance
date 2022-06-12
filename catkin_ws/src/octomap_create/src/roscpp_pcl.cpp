/*
 * PCL Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Include PCL visualizer
#include <pcl/visualization/cloud_viewer.h>


//Euclidean clustering tutorial
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//Octomap libraries
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>


#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

 // Topics
 static const std::string IMAGE_TOPIC = "/camera/depth_registered/points";
 static const std::string PUBLISH_TOPIC = "/pcl/points";
 static const std::string OCTOMAP_TOPIC = "octomap_full";


 // ROS Publisher
 ros::Publisher pub;
 ros::Publisher octo;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud (cloudPtr);
    vg.setLeafSize (0.05, 0.05, 0.05);
    vg.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);


    //Octomap pcl_conversions
    float octMapRes = 0.05;
    octomap::OcTree* octTree = new octomap::OcTree(octMapRes);
    octomap::Pointcloud octPointCloud;

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x"); it != it.end(); ++it) {
        octomap::point3d endpoint(it[0], it[1], it[2]);
        octPointCloud.push_back(endpoint);
    }
    octomap::point3d origin(0.0, 0.0, 0.0);
    octTree->insertPointCloud(octPointCloud, origin);
    octTree->updateInnerOccupancy();

    octTree->writeBinary("static_occ.bt");
    octomap_msgs::Octomap octomap;
    octomap.binary=1;
    octomap.id=1;
    octomap.resolution=0.05;
    octomap.header.frame_id = "camera_depth_optical_frame";
    octomap.header.stamp = ros::Time::now();
    bool res = octomap_msgs::fullMapToMsg(*octTree, octomap);

    octo.publish(octomap);
}



// Main function
int main(int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_example"
  ros::init(argc, argv, "pcl_listener");

  // Instantiate the ROS Node Handler as nh
  ros::NodeHandle nh;

  // Print "Hello ROS!" to the terminal and ROS log file
  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());


  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
  ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

  // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
  pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
  octo = nh.advertise<octomap_msgs::Octomap>(OCTOMAP_TOPIC,1,true);
  // Spin
  ros::spin();


  // Program succesful
  return 0;
}

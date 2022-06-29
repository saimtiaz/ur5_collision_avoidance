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
#include <pcl/common/io.h>

//Include PCL visualizer
#include <pcl/visualization/cloud_viewer.h>


//Euclidean clustering tutorial
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>



#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>




#include <vector>
#include <pcl/conversions.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include <pcl/PCLImage.h>
#include <sensor_msgs/Image.h>
#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl_msgs/PointIndices.h>

#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/Vertices.h>
#include <pcl_msgs/Vertices.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <iostream>
#include "std_msgs/Bool.h"

// Topics
static const std::string IMAGE_TOPIC = "/croppedPointCloud";
static const std::string SCAN_TOPIC = "/getNextScan";
static const std::string QUIT_TOPIC = "/quitScan";


bool getNextScan = true;
static float scanCount = 0;
//static pcl::PCLPointCloud2* finalCloud = new pcl::PCLPointCloud2;

 // ROS Publisher
 ros::Publisher pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO_STREAM("IN callback " << ros::this_node::getName());

    if(getNextScan == true)
    {
      ROS_INFO_STREAM("TAKING SCAN");
      getNextScan = false;

      //Read in cloud data
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
      
      //Create file name
      std::string dir = "scan";
      std::string fileExt = ".pcd";
      std::string fileName= dir + std::to_string(scanCount) + fileExt; 

      //Downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      vg.setInputCloud (temp_cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);

      //Save file
      pcl::io::savePCDFileASCII (fileName, *cloud_filtered); 
      //pcl::io::savePCDFileASCII (fileName, *temp_cloud);

      ROS_INFO_STREAM("SCAN COMPLETE");
      scanCount = scanCount + 1;
    }
}

void nextScan_cb(const std_msgs::Bool::ConstPtr& nextScan_msg)
{
    ROS_INFO_STREAM("In nextScan_cb " << nextScan_msg->data);
    if(getNextScan == false && nextScan_msg->data)
    {
    	getNextScan = true;
    }
    
}

void quit_cb(const std_msgs::Bool::ConstPtr& quit_msg)
{
    if(quit_msg->data == true)
    {
    	ros::shutdown();
    }
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
  ros::Subscriber cloudSub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

  ros::Subscriber nextScanSub = nh.subscribe(SCAN_TOPIC, 1, nextScan_cb);

  ros::Subscriber quitSub = nh.subscribe(QUIT_TOPIC, 1, quit_cb);


  ros::spin();


  // Program succesful
  return 0;
}

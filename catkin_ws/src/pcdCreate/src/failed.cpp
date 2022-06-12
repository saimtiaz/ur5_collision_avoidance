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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <tf/transform_listener.h>


#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include <pcl/PCLImage.h>
#include <sensor_msgs/Image.h>
#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/Vertices.h>
#include <pcl_msgs/Vertices.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
 // Topics
 //static const std::string IMAGE_TOPIC = "camera/depth/color/points";
static const std::string IMAGE_TOPIC = "/camera/depth_registered/points";



// ROS Publisher
ros::Publisher pub_;
ros::Subscriber sub_;
//ros::NodeHandle nh_;
tf::TransformListener listener_;
sensor_msgs::PointCloud2::Ptr buffer_;

static float callbackCount = 0;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO_STREAM("IN callback " << ros::this_node::getName());

  std::string frameIn;
  std::string frameOut;

  ros::param::get("/cloud_transformer/frameIn", frameIn);
  ros::param::get("/cloud_transformer/frameOut", frameOut);

  ROS_INFO("frameIn: %s, frameOut: %s", frameIn.c_str(), frameOut.c_str());

  listener_.waitForTransform(frameOut, frameIn, ros::Time::now(), ros::Duration(3.0));

  pcl_ros::transformPointCloud(frameOut, *cloud_msg, *buffer_, listener_);

  pub_.publish(buffer_);


  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*buffer_,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  callbackCount = callbackCount + 1;

  if(callbackCount < 2)
  {
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *temp_cloud);
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
  sub_ = nh.subscribe("/inputPC", 1, cloud_cb);
  pub_ = nh.advertise<sensor_msgs::PointCloud2>("/outputPC", 1);

  while (ros::ok())
  {
    ros::spin();
  }


  // Program succesful
  return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <string>

#include <pcl/io/pcd_io.h>


class CloudTransformer
{

public:
  explicit CloudTransformer(ros::NodeHandle nh)
    : nh_(nh)
  {
    // Define Publishers and Subscribers here
    pcl_sub_ = nh_.subscribe("/inputPC", 1, &CloudTransformer::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/outputPC", 1);

    std::string frameIn;
    std::string frameOut;
    ros::param::get("/cloud_transformer/frameIn", frameIn);
    ros::param::get("/cloud_transformer/frameOut", frameOut);
    ROS_INFO("frameIn: %s, frameOut: %s", frameIn.c_str(), frameOut.c_str());
    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = frameOut;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2::Ptr buffer_;
  float callbackCount = 0;


  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
  {
    
    std::string frameIn;
    std::string frameOut;
    ros::param::get("/cloud_transformer/frameIn", frameIn);
    ros::param::get("/cloud_transformer/frameOut", frameOut);
    ROS_INFO("frameIn: %s, frameOut: %s", frameIn.c_str(), frameOut.c_str());
    listener_.waitForTransform(frameOut, frameIn, ros::Time::now(), ros::Duration(3.0));
    pcl_ros::transformPointCloud(frameOut, *pcl_msg, *buffer_, listener_);
    pcl_pub_.publish(buffer_);
  }
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf");
  ros::NodeHandle nh;
  

  CloudTransformer transform_cloud(nh);

  // Spin until ROS is shutdown
  while (ros::ok())
    ros::spin();
  
  return 0;
}
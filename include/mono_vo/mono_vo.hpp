#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "mono_vo/feature_processor.hpp"
#include "mono_vo/initializer.hpp"
#include "mono_vo/map.hpp"
#include "mono_vo/tracker.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace mono_vo
{

class MonoVO : public rclcpp::Node
{
public:
  explicit MonoVO(const rclcpp::NodeOptions & options);

private:
  void setup();

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Frames
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string cam_frame_{"camera"};

  // Extrinsic base->camera
  cv::Affine3d T_base_cam_{cv::Affine3d::Identity()};
  bool has_extrinsic_{false};

  // Optional: for SE2 clamp using initial z
  double z0_{0.0};
  bool z0_set_{false};
  
  //------------------new improved lines------------------------------------------------

  Map::Ptr map_;
  FeatureProcessor::Ptr feature_processor_;
  Initializer initializer_;
  Tracker tracker_;

  std::optional<cv::Mat> K_;
  std::optional<cv::Mat> d_;

  nav_msgs::msg::Path path_msg_;
};

}  // namespace mono_vo

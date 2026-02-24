#include "mono_vo/mono_vo.hpp"

#include <cv_bridge/cv_bridge.h>
#include <functional>

#include "mono_vo/utils.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mono_vo {

MonoVO::MonoVO(const rclcpp::NodeOptions &options)
    : Node("mono_vo", options),
      map_(std::make_shared<Map>(this->get_logger().get_child("map"))),
      feature_processor_(std::make_shared<FeatureProcessor>(
          500, this->get_logger().get_child("feature_processor"))), // 1000
      initializer_(map_, feature_processor_,
                   this->get_logger().get_child("initializer")),
      tracker_(map_, feature_processor_,
               this->get_logger().get_child("tracker")) {
  this->setup();
}

void MonoVO::setup() {
  using std::placeholders::_1;

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/cam_1/image_raw", rclcpp::SensorDataQoS().keep_last(1).best_effort(),
      std::bind(&MonoVO::image_callback, this, _1));

  // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  // -----------------real one /////
  //   "/cam_1/image_raw", 10,
  //   rclcpp::SensorDataQoS().keep_last(1).best_effort(),
  //   [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  //   image_callback(msg); });

  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              image_sub_->get_topic_name());

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/cam_1/camera_info", rclcpp::QoS(1).reliable(),
      std::bind(&MonoVO::camera_info_callback, this, std::placeholders::_1));

  // camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
  // ---------------------------real one
  //   "/cam_1/camera_info", 10, [this](const
  //   sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
  //     camera_info_callback(msg);
  //   });
  //------------------------------------------------new data to convert
  // camera_pose to base_link-----------------------------------------------
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("camera_frame", "camera");

  // base->camera extrinsic as 6 numbers: x y z roll pitch yaw (meters, radians)
  this->declare_parameter<std::vector<double>>("T_base_cam_xyzrpy",
                                               {0, 0, 0, 0, 0, 0});

  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  cam_frame_ = this->get_parameter("camera_frame").as_string();

  auto ex = this->get_parameter("T_base_cam_xyzrpy").as_double_array();
  if (ex.size() == 6) {
    const double x = ex[0], y = ex[1], z = ex[2];
    const double rr = ex[3], pp = ex[4], yy = ex[5];

    cv::Matx33d Rx(1, 0, 0, 0, std::cos(rr), -std::sin(rr), 0, std::sin(rr),
                   std::cos(rr));
    cv::Matx33d Ry(std::cos(pp), 0, std::sin(pp), 0, 1, 0, -std::sin(pp), 0,
                   std::cos(pp));
    cv::Matx33d Rz(std::cos(yy), -std::sin(yy), 0, std::sin(yy), std::cos(yy),
                   0, 0, 0, 1);
    cv::Matx33d R = Rz * Ry * Rx;

    T_base_cam_ = cv::Affine3d(R, cv::Vec3d(x, y, z));
    has_extrinsic_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "T_base_cam_xyzrpy must have 6 values. Using identity.");
    T_base_cam_ = cv::Affine3d::Identity();
    has_extrinsic_ = true;
  }
  //------------------------------------------------new data to convert
  // camera_pose to base_link-----------------------------------------------
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'",
              camera_info_sub_->get_topic_name());

  //-----------------------new lines here---------------------------------------
  // auto clamp_to_se2 = [](const cv::Affine3d &T) {
  //   cv::Vec3d t = T.translation();
  //   cv::Matx33d R = T.rotation();

  //   // yaw from R (assuming base frame x forward, y left, z up)
  //   double yaw = std::atan2(R(1, 0), R(0, 0));

  //   cv::Matx33d Rz(std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw),
  //                  std::cos(yaw), 0, 0, 0, 1);

  //   // lock z to 0 (or keep initial z if you prefer)
  //   t[2] = 0.0;

  //   return cv::Affine3d(Rz, t);
  // };

  //---------------------------------------upto here-----------------------

  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
              odometry_pub_->get_topic_name());

  pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
              pointcloud_pub_->get_topic_name());

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'",
              path_pub_->get_topic_name());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto initializer_param_h = RosParameterHandler(this, "initializer");
  initializer_.configure_parameters(initializer_param_h);

  auto tracker_param_h = RosParameterHandler(this, "tracker");
  tracker_.configure_parameters(tracker_param_h);

  RCLCPP_INFO(this->get_logger(), "mono_vo node initialized");
}

void MonoVO::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

  // static int pc_counter = 0;                     ////////////////new lines 5
  // majorly if (++pc_counter % 5 == 0) {  // publish every 5 frames
  //   auto points = map_->get_landmark_points();
  //   auto pc_msg = utils::points3d_to_pointcloud_msg(points, header);
  //   pointcloud_pub_->publish(pc_msg);
  // }

  // std::cout<<"*****************Start here***********************"<<std::endl;
  RCLCPP_DEBUG(this->get_logger(), "Image message received at ts: '%d'",
               msg->header.stamp.sec);

  if (!K_.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Waiting for camera info to be published");
    return;
  }

  // std::cout<<"Control 1"<<std::endl;
  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = odom_frame_;

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // RCLCPP_INFO(this->get_logger(), "Incoming encoding: %s",
    // msg->encoding.c_str());     //Incoming encoding: bayer_rggb8 cv_ptr =
    // cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::bayer_rggb8);
    // //BGR8
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8"); // bgr8
    // Frame frame{cv_ptr->image}; ----------real
    // std::cout<<"Control 2"<<std::endl;

  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  Frame frame{cv_ptr->image};

  if (!initializer_.is_initalized()) {
    std::optional<Frame> ref_frame =
        initializer_.try_initializing(frame, K_.value());

    if (!ref_frame) {
      RCLCPP_INFO(this->get_logger(), "ref_frame = nullopt (not ready yet)");
      return;
    }

    if (ref_frame.has_value()) {
      // auto t0 = std::chrono::steady_clock::now();
      tracker_.update(ref_frame.value(), K_.value(), d_.value());
      // auto t1 = std::chrono::steady_clock::now();
      // double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "VO frame time:
      // %.1f ms", ms);
      RCLCPP_INFO(this->get_logger(), "Initialized");
      // std::cout<<"Control 4"<<std::endl;
    }
    return;
  }

  std::optional<cv::Affine3d> pose_wc =
      tracker_.update(frame, K_.value(), d_.value());

  cv::Affine3d T_odom_cam = pose_wc.value(); ///-----------------new lines
  cv::Affine3d T_cam_base = T_base_cam_.inv();
  cv::Affine3d T_odom_base = T_odom_cam * T_cam_base;

  // std_msgs::msg::Header header;
  // header.stamp = msg->header.stamp;
  // header.frame_id = odom_frame_;

  nav_msgs::msg::Odometry odom_msg =
      utils::affine3d_to_odometry_msg(T_odom_base, header, base_frame_);
  odometry_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped tf_msg =
      utils::affine3d_to_transform_stamped_msg(T_odom_base, header,
                                               base_frame_);
  tf_broadcaster_->sendTransform(tf_msg);

  //-------------------upto here-------------------------------

  if (tracker_.get_state() == TrackerState::LOST) {
    RCLCPP_WARN(this->get_logger(),
                "Tracker Lost -> clearing path (to avoid RViz spikes)");
    path_msg_.poses.clear();
    // RCLCPP_INFO(this->get_logger(), "Tracker Lost");
    // TODO (Myron): Add resetting logic
    return;
  }

  header.frame_id = "odom";
  //------------new lines--------------
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string cam_frame_ = "camera";
  //------------new lines--------------

  //--------------------------again new changes
  // here----------------------------------

  if (pose_wc.has_value()) {
    cv::Affine3d T_odom_cam = pose_wc.value();

    cv::Affine3d T_odom_base = T_odom_cam;
    if (has_extrinsic_) {
      cv::Affine3d T_cam_base = T_base_cam_.inv();
      T_odom_base = T_odom_cam * T_cam_base;
    }

    // clamp to ground (SE2)
    auto clamp_to_se2 = [this](const cv::Affine3d &T) {
      cv::Vec3d t = T.translation();
      cv::Matx33d R = T.rotation();
      double yaw = std::atan2(R(1, 0), R(0, 0));

      cv::Matx33d Rz(std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw),
                     std::cos(yaw), 0, 0, 0, 1);

      if (!z0_set_) {
        z0_ = t[2];
        z0_set_ = true;
      }
      t[2] = z0_; // or 0.0

      return cv::Affine3d(Rz, t);
    };

    T_odom_base = clamp_to_se2(T_odom_base);

    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = odom_frame_;

    auto odom_msg =
        utils::affine3d_to_odometry_msg(T_odom_base, header, base_frame_);
    odometry_pub_->publish(odom_msg);

    auto tf_msg = utils::affine3d_to_transform_stamped_msg(T_odom_base, header,
                                                           base_frame_);
    tf_broadcaster_->sendTransform(tf_msg);

    // path
    path_msg_.header = header;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose = odom_msg.pose.pose;
    path_msg_.poses.push_back(ps);
    path_pub_->publish(path_msg_);

    // pointcloud throttle (optional)
    static int pc_counter = 0;
    if (++pc_counter % 5 == 0) {
      auto points = map_->get_landmark_points();
      auto pc_msg = utils::points3d_to_pointcloud_msg(points, header);
      pointcloud_pub_->publish(pc_msg);
    }
  }
}
//   if (pose_wc.has_value()) {
//     nav_msgs::msg::Odometry odometry_msg =
//       utils::affine3d_to_odometry_msg(pose_wc.value(), header, "camera");
//     odometry_pub_->publish(odometry_msg);

//     geometry_msgs::msg::TransformStamped transform_msg =
//       utils::affine3d_to_transform_stamped_msg(pose_wc.value(), header,
//       "camera");
//     tf_broadcaster_->sendTransform(transform_msg);

//     path_msg_.header = header;
//     geometry_msgs::msg::PoseStamped current_pose_stamped;
//     current_pose_stamped.pose = odometry_msg.pose.pose;
//     current_pose_stamped.header = header;
//     path_msg_.poses.push_back(current_pose_stamped);
//     path_pub_->publish(path_msg_);
//   }

//   // publish pointcloud from map
//   std::vector<cv::Point3f> points = map_->get_landmark_points();
//   sensor_msgs::msg::PointCloud2 pointcloud_msg =
//   utils::points3d_to_pointcloud_msg(points, header);
//   pointcloud_pub_->publish(pointcloud_msg);
// }

void MonoVO::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
  RCLCPP_DEBUG(this->get_logger(), "Camera info message received at ts: '%d'",
               msg->header.stamp.sec);
  if (K_.has_value() && d_.has_value())
    return;

  K_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
  d_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(msg->d.data())).clone();
}
} // namespace mono_vo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mono_vo::MonoVO)
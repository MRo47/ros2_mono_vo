#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include <vector>

#include "mono_vo/feature_processor.hpp"
#include "mono_vo/frame.hpp"
#include "mono_vo/map.hpp"
#include "mono_vo/match_data.hpp"
#include "mono_vo/ros_parameter_handler.hpp"
#include "mono_vo/utils.hpp"

namespace mono_vo
{

class Initializer
{
public:
  enum class State
  {
    OBTAINING_REF,
    INITIALIZING,
    INITIALIZED
  };

  Initializer(
    Map::Ptr map,
    FeatureProcessor::Ptr feature_extractor = std::make_shared<FeatureProcessor>(1000),
    rclcpp::Logger logger = rclcpp::get_logger("Initializer"));

  void configure_parameters(RosParameterHandler & reader);

  bool is_initalized();

  void reset();

  /**
   * Evaluates the distribution of keypoints across a frame to ensure a good spread.
   *
   * This function checks if the keypoints in the provided frame are well-distributed
   * by dividing the image into a grid and counting the number of grid cells occupied
   * by at least one keypoint. It calculates the occupancy ratio of occupied cells to
   * total cells and compares it against a predefined threshold to determine if the
   * distribution is sufficient.
   *
   * @param frame The frame containing keypoints to evaluate.
   * @return True if the keypoint distribution is above the threshold, otherwise false.
   */
  bool good_keypoint_distribution(const Frame & frame);

  /**
   * Check if the parallax between two sets of points is sufficient.
   *
   * This function calculates the score of the homography and fundamental matrix
   * between two sets of points. It checks if the score of the fundamental matrix
   * is above a certain threshold, and if the ratio of the score of the homography
   * to the score of the fundamental matrix is below a certain threshold. If both
   * conditions are met, the function returns true. Otherwise, it returns false.
   *
   * @param pts1 The first set of points.
   * @param pts2 The second set of points.
   * @return True if the parallax is sufficient, otherwise false.
   */
  bool check_parallax(const std::vector<cv::Point2f> & pts1, const std::vector<cv::Point2f> & pts2);

  /**
   * Triangulate points given reference and current frames.
   *
   * The function takes two sets of 2D points, ref_points and cur_points, and computes the corresponding 3D points.
   * The points are triangulated using the projection matrices P_ref and P_cur, and the inliers are
   * checked using a chirality check.
   *
   * The output is a 3d point vector, whose size() is the number of valid 3D points.
   *
   * @param K The camera intrinsic matrix.
   * @param R The rotation matrix from the reference frame to the current frame.
   * @param t The translation vector from the reference frame to the current frame.
   * @param ref_points The 2D points in the reference frame.
   * @param cur_points The 2D points in the current frame.
   * @param inliers The inliers of the triangulation.
   *
   * @return A 3d point vector.
   */
  std::vector<cv::Point3f> traingulate_points(
    const cv::Mat & K, const cv::Mat & R_cw, const cv::Mat & t_cw,
    const std::vector<cv::Point2f> & ref_points, const std::vector<cv::Point2f> & cur_points,
    std::vector<uchar> & inliers);

  /**
   * @brief Try to initialize the VO system using the given frame and K matrix.
   * The state machine is as follows:
   *  - State::OBTAINING_REF: Get a frame with a good keypoint distribution.
   *  - State::INITIALIZING: Find a frame with enough matches and pass parallax check.
   *  - State::INITIALIZED: The VO system is now initialized and the first two frames are added.
   * @param frame the current frame
   * @param K the camera intrinsic matrix
   * @return the reference frame if the system is now initialized, else std::nullopt
   */
  std::optional<Frame> try_initializing(const Frame & frame, const cv::Mat & K);

private:
  Map::Ptr map_;
  rclcpp::Logger logger_;
  State state_;
  Frame ref_frame_;
  int occupancy_grid_div_ = 64;             // NxN grid size to check for good keypoint distribution
  double kp_distribution_thresh_ = 0.15;     // threshold for good keypoint distribution
  double lowes_distance_ratio_ = 0.7;       // Lowe's distance ratio for finding good matches
  int64_t min_matches_for_init_ = 100;      // minimum number of matches for initialization
  double ransac_reproj_thresh_ = 1.0;       // px RANSAC threshold for fitting H/F model
  double f_inlier_thresh_ = 0.5;            // fundamental inlier threshold ratio
  double model_score_thresh_ = 0.56;        // max H/F ratio
  double current_min_model_score_ = 100.0;  // current min H/F ratio for debug

  FeatureProcessor::Ptr feature_processor_;
};
}  // namespace mono_vo
#include "mono_vo/initializer.hpp"

#include "mono_vo/ros_parameter_handler.hpp"

#include <cmath>     
#include <algorithm>

namespace mono_vo
{

Initializer::Initializer(
  Map::Ptr map, FeatureProcessor::Ptr feature_processor, rclcpp::Logger logger)
: map_(map),
  logger_(logger),
  state_(State::OBTAINING_REF),
  ref_frame_(cv::Mat()),
  feature_processor_(feature_processor)
{
}

void Initializer::configure_parameters(RosParameterHandler & param_handler)
{
  param_handler.declare_and_get(
    "occupancy_grid_div", occupancy_grid_div_, "Grid size (N) for checking keypoint distribution.");

  param_handler.declare_and_get(
    "kp_distribution_thresh", kp_distribution_thresh_,
    "Minimum ratio of grid cells that must contain keypoints.");

  param_handler.declare_and_get(
    "lowes_distance_ratio", lowes_distance_ratio_,
    "Lowe's ratio for feature matching; lower is stricter.");

  param_handler.declare_and_get(
    "min_matches_for_init", min_matches_for_init_,
    "Minimum matches needed to attempt VO initialization.");

  param_handler.declare_and_get(
    "ransac_reproj_thresh", ransac_reproj_thresh_,
    "RANSAC reprojection threshold (pixels) for H/F model fitting.");

  param_handler.declare_and_get(
    "f_inlier_thresh", f_inlier_thresh_,
    "Minimum inlier ratio to accept the Fundamental matrix model.");

  param_handler.declare_and_get(
    "model_score_thresh", model_score_thresh_,
    "Score ratio (H/F) threshold to decide between planar and general scene models.");
}

bool Initializer::is_initalized() { return state_ == State::INITIALIZED; }

void Initializer::reset() { state_ = State::OBTAINING_REF; }

//----------------------------------changed good initializer----------------------------------------

bool Initializer::good_keypoint_distribution(const Frame& frame)
{
  if (frame.image.empty()) {
    RCLCPP_WARN(logger_, "good_dist: empty image");
    return false;
  }

  if (occupancy_grid_div_ <= 0) {
    RCLCPP_WARN(logger_, "good_dist: occupancy_grid_div_ must be > 0 (got %d)", occupancy_grid_div_);
    return false;
  }

  const int H = frame.image.rows;
  const int W = frame.image.cols;

  const int grid_rows = std::max(1, H / occupancy_grid_div_);
  const int grid_cols = std::max(1, W / occupancy_grid_div_);
  cv::Mat grid = cv::Mat::zeros(grid_rows, grid_cols, CV_8U);

  int occupied_cells = 0;
  int oob = 0;

  for (const auto& obs : frame.observations) {
    const int r = (int)obs.keypoint.pt.y / occupancy_grid_div_;
    const int c = (int)obs.keypoint.pt.x / occupancy_grid_div_;

    if (r < 0 || r >= grid_rows || c < 0 || c >= grid_cols) { oob++; continue; }

    if (!grid.at<uchar>(r, c)) {
      grid.at<uchar>(r, c) = 1;
      occupied_cells++;
    }
  }

  const int total_cells = grid_rows * grid_cols;
  const double occupancy = (double)occupied_cells / (double)total_cells;

  RCLCPP_INFO(logger_,
    "good_dist: obs=%zu grid=%dx%d occ=%d/%d (%.3f) oob=%d div=%d thresh=%.3f",
    frame.observations.size(), grid_rows, grid_cols,
    occupied_cells, total_cells, occupancy, oob,
    occupancy_grid_div_, kp_distribution_thresh_);

  return occupancy > kp_distribution_thresh_;
}
//-----------------------------------------------------------------------------------------------

//-------------------------------main code -----------------------------------------------------

// bool Initializer::good_keypoint_distribution(const Frame & frame)
// {
//   RCLCPP_INFO(logger_, "totals kps: %ld", frame.observations.size());

//   // check distribution in a grid across the image
//   cv::Mat grid = cv::Mat::zeros(
//     frame.image.rows / occupancy_grid_div_, frame.image.cols / occupancy_grid_div_, CV_8U);
//   int occupied_cells = 0;
//   for (auto & obs : frame.observations) {
//     int r = obs.keypoint.pt.y / occupancy_grid_div_, c = obs.keypoint.pt.x / occupancy_grid_div_;
//     if (!grid.at<uchar>(r, c)) {
//       grid.at<uchar>(r, c) = 1;
//       occupied_cells++;
//     }
//   }
//   auto total_cells = grid.cols * grid.rows;
//   RCLCPP_INFO(logger_, "occupied cells: %d total cells: %d", occupied_cells, total_cells);
//   auto occupancy = static_cast<double>(occupied_cells) / total_cells;
//   RCLCPP_INFO(logger_, "occupancy: %lf", occupancy);
//   if (occupancy > kp_distribution_thresh_) {
//     return true;
//   }
//   return false;
// }

//-------------------------------------------------------------------------------------------------------


bool Initializer::check_parallax(
  const std::vector<cv::Point2f> & pts1, const std::vector<cv::Point2f> & pts2)
{
  // calculate homography
  std::vector<uchar> inliers_h;
  cv::findHomography(pts1, pts2, cv::RANSAC, ransac_reproj_thresh_, inliers_h);
  int score_h = cv::countNonZero(inliers_h);

  // calculate fundamental
  std::vector<uchar> inliers_f;
  cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, ransac_reproj_thresh_, 0.99, inliers_f);
  int score_f = cv::countNonZero(inliers_f);

  RCLCPP_INFO(logger_, "score h: %d, score f: %d", score_h, score_f);
  // check 1: min inliers
  if (static_cast<double>(score_f) / pts1.size() < f_inlier_thresh_) {
    return false;
  }

  auto model_score = static_cast<double>(score_h) / static_cast<double>(score_f);
  RCLCPP_INFO(logger_, "score_h/score_f = %lf", model_score);

  current_min_model_score_ = std::min(current_min_model_score_, model_score);
  RCLCPP_INFO(logger_, "current_min_model_score = %lf", current_min_model_score_);

  // check 2: Is the Fundamental Matrix a significantly better model?
  // The ratio score_H / score_F should be low.
  // A high ratio means Homography explains the data almost as well as Fundamental Matrix.
  if (model_score > model_score_thresh_) {
    return false;
  }

  return true;
}

std::vector<cv::Point3f> Initializer::traingulate_points(
  const cv::Mat & K, const cv::Mat & R_cw, const cv::Mat & t_cw,
  const std::vector<cv::Point2f> & ref_points, const std::vector<cv::Point2f> & cur_points,
  std::vector<uchar> & inliers)
{
  // get projection matrices
  cv::Mat P_ref = K * cv::Mat::eye(3, 4, CV_64F);  // P_ref = K * [I | 0]

  cv::Mat Rt_cw;
  cv::hconcat(R_cw, t_cw, Rt_cw);
  cv::Mat P_cur = K * Rt_cw;  // P_cur = K * [R | t]

  cv::Mat pts4d_h;  // Output is 4xN matrix of homogeneous 3D points
  cv::triangulatePoints(P_ref, P_cur, ref_points, cur_points, pts4d_h);

  RCLCPP_INFO(logger_, "Triangulated %d 3D points", pts4d_h.cols);

  // convert to cartesian
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts4d_h.t(), pts3d);

  // chirality check:check if point is in front of both reference and current camera frame
  auto is_infront = [&R_cw, &t_cw](const cv::Point3f & p3d) {
    // check if point is in front of the reference camera
    if (p3d.z <= 0) {
      return false;
    }

    // Transform the 3D point into the current camera's coordinate system.
    cv::Mat p3d_mat = (cv::Mat_<double>(3, 1) << p3d.x, p3d.y, p3d.z);
    cv::Mat p3d_cur_mat = R_cw * p3d_mat + t_cw;
    return p3d_cur_mat.at<double>(2, 0) > 0;
  };

  inliers.clear();
  inliers.reserve(pts3d.rows);
  std::vector<cv::Point3f> points_3d;
  for (int i = 0; i < pts3d.rows; ++i) {
    const cv::Point3f & p3d_ref = pts3d.at<cv::Point3f>(i);
    if (is_infront(p3d_ref)) {
      inliers.push_back(1);
      points_3d.push_back(p3d_ref);
    } else {
      inliers.push_back(0);
    }
  }

  RCLCPP_INFO(
    logger_, "Triangulation complete. Kept %zu / %d valid points.", points_3d.size(), pts3d.rows);

  return points_3d;
}

std::optional<Frame> Initializer::try_initializing(const Frame & frame, const cv::Mat & K)
{
  if (state_ == State::INITIALIZED) {
    return ref_frame_;
  }

  Frame cur_frame{frame};
  cur_frame.extract_observations(feature_processor_);
  // RCLCPP_WARN(logger_, "INIT: kp=%zu desc=%d x %d",     //extra added line
  //           cur_frame.observations.size(),
  //           cur_frame.get_descriptors().rows,
  //           cur_frame.get_descriptors().cols);

  if (state_ == State::OBTAINING_REF) {
    
    if (!good_keypoint_distribution(cur_frame)) {
      RCLCPP_WARN(logger_, "INIT: ref rejected: bad keypoint distribution"); //added extra line ------main problem?
    return std::nullopt;
    }
    RCLCPP_INFO(logger_, "found good reference frame");
    ref_frame_ = std::move(cur_frame);
    state_ = State::INITIALIZING;
    return std::nullopt;
  }

  if (state_ == State::INITIALIZING) {
    std::vector<std::vector<cv::DMatch>> knn_matches;
    auto ref_descriptors = ref_frame_.get_descriptors();
    auto cur_descriptors = cur_frame.get_descriptors();

    std::vector<cv::DMatch> good_matches =
      feature_processor_->find_matches(ref_descriptors, cur_descriptors, lowes_distance_ratio_);

    if (static_cast<int64_t>(good_matches.size()) < min_matches_for_init_) {
      RCLCPP_WARN(logger_, "Initializer: Not enough matches");
      // check if new frame is good ref, if yes set it else continue initializing step
      if (good_keypoint_distribution(cur_frame)) {
        RCLCPP_WARN(logger_, "found new good reference frame");
        ref_frame_ = std::move(cur_frame);
      } else {
        RCLCPP_WARN(logger_, "Resetting state");
        reset();
      }
      return std::nullopt;
    }

    std::vector<MatchData> all_matches;
    all_matches.reserve(good_matches.size());
    for (const auto & match : good_matches) {
      all_matches.emplace_back(
        match.queryIdx, match.trainIdx, ref_frame_.observations[match.queryIdx].keypoint.pt,
        cur_frame.observations[match.trainIdx].keypoint.pt);
    }

    // Extract points for parallax check
    auto [pts_ref_matched, pts_cur_matched] = extract_points_from_matches(all_matches);

    cv::Mat img_matches = utils::draw_matched_points(
      ref_frame_.image, cur_frame.image, pts_ref_matched, pts_cur_matched);
    cv::imshow("Matches", img_matches);
    cv::waitKey(1);

    if (!check_parallax(pts_ref_matched, pts_cur_matched)) {
      RCLCPP_WARN(logger_, "Parallax check failed");
      return std::nullopt;
    }
    RCLCPP_INFO(logger_, "Parallax check passed");

    // find essential matrix
    std::vector<uchar> inlier_mask_E;
    cv::Mat E = cv::findEssentialMat(
      pts_ref_matched, pts_cur_matched, K, cv::RANSAC, 0.99, 1.0, inlier_mask_E);
    RCLCPP_INFO(
      logger_, "Found Essential Matrix with inlier ratio %lf",
      static_cast<double>(cv::countNonZero(inlier_mask_E)) / pts_ref_matched.size());

    // decompose to get rotation and translation
    cv::Mat R_cw, t_cw;
    cv::recoverPose(E, pts_ref_matched, pts_cur_matched, K, R_cw, t_cw, inlier_mask_E);

    uint32_t num_inliers = cv::countNonZero(inlier_mask_E);

    // print R,t and inliers
    RCLCPP_INFO(
      logger_, "Inlier ratio after pose recovery: %lf",
      static_cast<double>(num_inliers) / pts_ref_matched.size());

    // filter points
    std::vector<MatchData> pose_inliers;
    pose_inliers.reserve(cv::countNonZero(inlier_mask_E));
    for (size_t i = 0; i < all_matches.size(); ++i) {
      if (inlier_mask_E[i]) {
        pose_inliers.push_back(all_matches[i]);
      }
    }

    // triangulate points
    auto [pts_ref_inliers, pts_cur_inliers] = extract_points_from_matches(pose_inliers);

    std::vector<uchar> chirality_mask;
    std::vector<cv::Point3f> pts3d =
      traingulate_points(K, R_cw, t_cw, pts_ref_inliers, pts_cur_inliers, chirality_mask);

    // check min 4 points are valid for PnP later
    if (pts3d.size() < 4) {
      RCLCPP_WARN(logger_, "Less than 4 points triangulated: resetting initializer");
      reset();
      return std::nullopt;
    }

    // add origin keyframe
    KeyFrame::Ptr origin_keyframe =
      std::make_shared<KeyFrame>(cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d::zeros()));
    map_->add_keyframe(origin_keyframe);

    // set frame pose
    cur_frame.pose_wc = cv::Affine3d(R_cw, t_cw).inv();

    // Create landmarks and update landmark_id in the original frames using the final data.
    int valid_pts3d_idx = 0;
    for (size_t i = 0; i < pose_inliers.size(); ++i) {
      if (chirality_mask[i]) {
        // Get the correspondence data and the 3D point
        const MatchData & data = pose_inliers[i];
        const cv::Point3f & p3d = pts3d[valid_pts3d_idx++];

        // Create the landmark using the descriptor from the current frame
        Landmark lm = Landmark{p3d, cur_frame.observations[data.cur_idx].descriptor};
        map_->add_landmark(lm);

        // Update the landmark_id in BOTH frames using the original indices
        cur_frame.observations[data.cur_idx].landmark_id = lm.id;
        ref_frame_.observations[data.ref_idx].landmark_id = lm.id;
      }
    }

    map_->add_keyframe(std::make_shared<KeyFrame>(cur_frame));

    img_matches = utils::draw_matched_points(
      ref_frame_.image, cur_frame.image, pts_ref_inliers, pts_cur_inliers);
    cv::imshow("Matches", img_matches);
    cv::waitKey(1);

    ref_frame_ = cur_frame;
    state_ = State::INITIALIZED;
    return ref_frame_;
  }
  return std::nullopt;
}
}
 // namespace mono_vo
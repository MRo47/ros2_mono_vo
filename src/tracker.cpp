#include "mono_vo/tracker.hpp"

#include <sstream>

#include "mono_vo/match_data.hpp"

namespace mono_vo
{

Tracker::Tracker(Map::Ptr map, FeatureProcessor::Ptr feature_extractor, rclcpp::Logger logger)
: map_(map), prev_frame_(cv::Mat()), feature_processor_(feature_extractor), logger_(logger)
{
}

void Tracker::configure_parameters(RosParameterHandler & param_handler)
{
  param_handler.declare_and_get(
    "tracking_error_thresh", tracking_error_thresh_,
    "Lucas-Kanade (LK) optical flow tracking error threshold in pixels.");

  param_handler.declare_and_get(
    "min_observations_before_triangulation", min_observations_before_triangulation_,
    "Keyframe trigger: minimum total feature observations required.");

  param_handler.declare_and_get(
    "min_tracked_points", min_tracked_points_,
    "Minimum tracked points required; tracking is declared lost if count falls below this.");

  param_handler.declare_and_get(
    "max_tracking_after_keyframe", max_tracking_after_keyframe_,
    "Keyframe trigger: maximum number of frames to track after the last keyframe.");

  param_handler.declare_and_get(
    "max_rotation_from_keyframe", max_rotation_from_keyframe_,
    "Keyframe trigger: maximum rotation (radians) from the last keyframe.");

  param_handler.declare_and_get(
    "max_translation_from_keyframe", max_translation_from_keyframe_,
    "Keyframe trigger: maximum translation (meters) from the last keyframe.");

  param_handler.declare_and_get(
    "ransac_reproj_thresh", ransac_reproj_thresh_,
    "RANSAC reprojection threshold (pixels) for estimating pose.");

  param_handler.declare_and_get(
    "model_score_thresh", model_score_thresh_,
    "H/F model score threshold for robust pose estimation.");

  param_handler.declare_and_get(
    "f_inlier_thresh", f_inlier_thresh_,
    "Minimum inlier ratio to accept the Fundamental matrix model for pose estimation.");

  param_handler.declare_and_get(
    "lowes_distance_ratio", lowes_distance_ratio_,
    "Lowe's ratio for matching new features against the last keyframe.");
}

Frame Tracker::track_frame_with_optical_flow(const cv::Mat & new_image)
{
  Frame new_frame{new_image};
  auto prev_pts_2d = prev_frame_.get_points_2d(ObservationFilter::WITH_LANDMARKS);
  auto prev_observations = prev_frame_.get_observations(ObservationFilter::WITH_LANDMARKS);
  std::vector<cv::Point2f> new_pts_2d;
  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point2f> prev_pts_2d_filtered;

  cv::calcOpticalFlowPyrLK(
    prev_frame_.image, new_frame.image, prev_pts_2d, new_pts_2d, status, err);
  for (size_t i = 0; i < prev_pts_2d.size(); ++i) {
    if (status[i] && err[i] < tracking_error_thresh_) {
      prev_pts_2d_filtered.push_back(prev_pts_2d[i]);  // TODO: remove, for debug only
      new_frame.add_observation(
        cv::KeyPoint(new_pts_2d[i], 1), prev_observations[i].descriptor,
        prev_observations[i].landmark_id);
    }
  }

  RCLCPP_INFO(logger_, "tracked %zu points", new_frame.observations.size());

  // TODO: remove, for debug only
  cv::Mat img_matches = utils::draw_matched_points(
    prev_frame_.image, new_frame.image, prev_pts_2d_filtered,
    new_frame.get_points_2d(ObservationFilter::ALL));
  // cv::imshow("Matches", img_matches); -----------changes commented these lines
  // cv::waitKey(1);

  new_frame.is_tracked = true;
  return new_frame;
}

bool Tracker::has_significant_motion(const Frame & frame)
{
  // get relative pose
  KeyFrame::Ptr prev_kframe = map_->get_last_keyframe();
  cv::Affine3d relative_pose = prev_kframe->pose_wc.inv() * frame.pose_wc;

  double translation = cv::norm(relative_pose.translation());

  if (translation > max_translation_from_keyframe_) {
    RCLCPP_WARN(logger_, "translation exceeds threshold: %lf", translation);
    return true;
  }
  // Convert the rotation matrix to an axis-angle rotation vector
  cv::Matx33d R = relative_pose.rotation();
  double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double rotation = std::acos((trace - 1.0) / 2.0);  // angle in radians

  // The magnitude of the rotation vector is the angle in radians
  if (rotation > max_rotation_from_keyframe_) {
    RCLCPP_WARN(logger_, "rotation exceeds threshold: %lf", rotation);
    return true;
  }

  return false;
}

bool Tracker::should_add_keyframe(const Frame & frame)
{
  if (static_cast<int64_t>(frame.observations.size()) < min_observations_before_triangulation_) {
    RCLCPP_WARN(logger_, "Not enough 3D points for triangulation");
    return true;
  }

  if (static_cast<int64_t>(tracking_count_from_keyframe_) > max_tracking_after_keyframe_) {
    RCLCPP_WARN(logger_, "Not enough tracking after keyframe");
    return true;
  }

  if (has_significant_motion(frame)) {
    RCLCPP_WARN(logger_, "Significant motion");
    return true;
  }

  return false;
}

std::vector<cv::Point3f> Tracker::triangulate_points(
  const cv::Affine3d & pose_ref_cw, const cv::Affine3d & pose_cur_cw, const cv::Mat & K,
  const std::vector<cv::Point2f> & pts_ref, const std::vector<cv::Point2f> & pts_cur,
  std::vector<uchar> & inliers)
{
  cv::Mat extrinsics_ref = cv::Mat(pose_ref_cw.matrix)(cv::Rect(0, 0, 4, 3));
  cv::Mat extrinsics_cur = cv::Mat(pose_cur_cw.matrix)(cv::Rect(0, 0, 4, 3));
  cv::Mat P_ref = K * extrinsics_ref;
  cv::Mat P_cur = K * extrinsics_cur;

  cv::Mat pts4d_h;  // 4xN homogeneous points
  cv::triangulatePoints(P_ref, P_cur, pts_ref, pts_cur, pts4d_h);

  std::vector<cv::Point3f> pts3d;
  cv::convertPointsFromHomogeneous(pts4d_h.t(), pts3d);

  // chirality check: point is in front of both reference and current camera frame
  auto is_infront = [&pose_ref_cw, &pose_cur_cw](const cv::Point3f & p3d) {
    // transform point to each camera frame
    cv::Point3f p3d_ref = pose_ref_cw * p3d;
    cv::Point3f p3d_cur = pose_cur_cw * p3d;

    return p3d_ref.z > 0 && p3d_cur.z > 0;
  };

  inliers.clear();
  inliers.reserve(pts3d.size());
  std::vector<cv::Point3f> pts_3d_inliers;
  for (const auto & p3d : pts3d) {
    if (is_infront(p3d)) {
      inliers.push_back(1);
      pts_3d_inliers.push_back(p3d);
    } else {
      inliers.push_back(0);
    }
  }

  RCLCPP_INFO(
    logger_, "Triangulation complete. Kept %zu / %zu valid points.", pts_3d_inliers.size(),
    pts3d.size());

  return pts_3d_inliers;
}

void Tracker::add_new_keyframe(Frame & frame, const cv::Mat & K)
{
  // detect new features from frame
  frame.clear_observations();
  frame.extract_observations(feature_processor_);

  // find matches with the previous keyframe points that dont have landmarks.
  auto prev_kframe = map_->get_last_keyframe();
  std::vector<cv::DMatch> good_matches = feature_processor_->find_matches(
    prev_kframe->get_descriptors(), frame.get_descriptors(), lowes_distance_ratio_);

  std::vector<MatchData> good_matches_data;
  good_matches_data.reserve(good_matches.size());
  for (const auto & match : good_matches) {
    good_matches_data.emplace_back(
      match.queryIdx, match.trainIdx, prev_kframe->observations[match.queryIdx].keypoint.pt,
      frame.observations[match.trainIdx].keypoint.pt);
  }

  auto [pts_ref_matched, pts_cur_matched] = extract_points_from_matches(good_matches_data);

  // use recovered pose from PnP and previous keyframe pose to get projection matrices
  cv::Affine3d pose_ref_cw = prev_kframe->pose_wc.inv();
  cv::Affine3d pose_cur_cw = frame.pose_wc.inv();

  std::vector<uchar> chirality_mask;
  std::vector<cv::Point3f> pts_3d = triangulate_points(
    pose_ref_cw, pose_cur_cw, K, pts_ref_matched, pts_cur_matched, chirality_mask);

  // triangulate new 3D points and add to map
  size_t valix_pts3d_idx = 0;
  for (size_t i = 0; i < chirality_mask.size(); ++i) {
    if (chirality_mask[i]) {
      const MatchData & data = good_matches_data[i];
      const cv::Point3f & p3d = pts_3d[valix_pts3d_idx++];

      // if kf prev has landmark, then set this same id for current frame
      if (long lid = prev_kframe->observations[data.ref_idx].landmark_id; lid != -1) {
        frame.observations[data.cur_idx].landmark_id = lid;
      } else {  // add as new landmark and set in both frames
        Landmark lm = Landmark(p3d, frame.observations[data.cur_idx].descriptor);
        map_->add_landmark(lm);
        frame.observations[data.cur_idx].landmark_id = lm.id;
        prev_kframe->observations[data.ref_idx].landmark_id = lm.id;
      }
    }
  }

  // set new keyframe with all detected keypoints
  map_->add_keyframe(std::make_shared<KeyFrame>(frame));

  // reset the tracking count
  tracking_count_from_keyframe_ = 0;
}

bool Tracker::has_parallax(const Frame & frame)
{
  auto pts1 = map_->get_last_keyframe()->get_points_2d_for_landmarks(frame.get_landmark_ids());
  auto pts2 = frame.get_points_2d();
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

  // check 2: Is the Fundamental Matrix a significantly better model?
  // The ratio score_H / score_F should be low.
  // A high ratio means Homography explains the data almost as well as Fundamental Matrix.
  if (model_score > model_score_thresh_) {
    return false;
  }

  return true;
}

TrackerState Tracker::get_state() const { return state_; }

void Tracker::reset() { state_ = TrackerState::INITIALIZING; }

std::optional<cv::Affine3d> Tracker::update(
  const Frame & frame, const cv::Mat & K, const cv::Mat & d)
{
  if (state_ == TrackerState::LOST) {
    return std::nullopt;
  }

  // set first frame
  if (state_ == TrackerState::INITIALIZING) {
    prev_frame_ = frame;
    state_ = TrackerState::TRACKING;
    return std::nullopt;
  }

  // --- track points with optical flow ---
  Frame new_frame = track_frame_with_optical_flow(frame.image);

  // check if enough points were tracked
  if (static_cast<int64_t>(new_frame.observations.size()) < min_tracked_points_) {
    RCLCPP_WARN(logger_, "Not enough keypoints were tracked");
    state_ = TrackerState::LOST;
    return std::nullopt;
  }

  // --- solve PnP ---
  // get new frame tracked 2D and corresponding 3D points
  auto [new_2dps, new_3dps] =
    map_->get_observation_to_landmark_point_correspondences(new_frame.observations);

  RCLCPP_INFO(logger_, "Got %zu 3D point correspondences from tracking", new_3dps.size());

  // Transform that brings point in world to camera frame
  cv::Mat rvec_cw;  // rotation vector in 3x1 format (Rodrigues format)
  cv::Mat tvec_cw;  // translation vector in 3x1 format
  cv::Mat inliers;  // indices of inliers
  cv::solvePnPRansac(new_3dps, new_2dps, K, d, rvec_cw, tvec_cw, false, 100, 8.0, 0.99, inliers);

  RCLCPP_INFO(logger_, "Solved PnP with %d inliers", inliers.rows);

  // get camera pose in world frame
  cv::Mat R_cw;
  cv::Rodrigues(rvec_cw, R_cw);
  new_frame.pose_wc = cv::Affine3d(R_cw, tvec_cw).inv();

  tracking_count_from_keyframe_++;
  if (should_add_keyframe(new_frame)) {
    if (has_parallax(new_frame)) {
      RCLCPP_INFO(logger_, "Has enough parallax, adding keyframe");
      add_new_keyframe(new_frame, K);
    }
  }

  std::stringstream ss;
  ss << "Camera pose transform wc: \n" << new_frame.pose_wc.matrix << std::endl;
  RCLCPP_INFO(logger_, "%s", ss.str().c_str());

  // update last frame
  prev_frame_ = std::move(new_frame);
  return prev_frame_.pose_wc;
}

}  // namespace mono_vo
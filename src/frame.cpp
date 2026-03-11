#include "mono_vo/frame.hpp"

namespace mono_vo
{

Frame::Frame(const cv::Mat & image) : image(image.clone()) {}

void Frame::extract_observations(const FeatureProcessor::Ptr feature_processor)
{
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  feature_processor->detect_and_compute(image, keypoints, descriptors);
  observations.reserve(keypoints.size());
  for (size_t i = 0; i < keypoints.size(); i++) {
    add_observation(keypoints[i], descriptors.row(i), -1);
  }
}

void Frame::add_observation(
  const cv::KeyPoint & keypoint, const cv::Mat & descriptor, long landmark_id)
{
  observations.emplace_back(keypoint, descriptor, landmark_id);
}

std::vector<cv::Point2f> Frame::get_points_2d(ObservationFilter filter_type) const
{
  std::vector<cv::Point2f> points_2d;
  points_2d.reserve(observations.size());
  for (const auto & obs : observations) {
    if (
      filter_type == ObservationFilter::ALL ||
      (filter_type == ObservationFilter::WITH_LANDMARKS && obs.landmark_id != -1) ||
      (filter_type == ObservationFilter::WITHOUT_LANDMARKS && obs.landmark_id == -1)) {
      points_2d.push_back(obs.keypoint.pt);
    }
  }
  return points_2d;
}

std::vector<long> Frame::get_landmark_ids() const
{
  std::vector<long> landmark_ids;
  landmark_ids.reserve(observations.size());
  for (const auto & obs : observations) {
    landmark_ids.push_back(obs.landmark_id);
  }
  return landmark_ids;
}

cv::Mat Frame::get_descriptors() const
{
  cv::Mat descriptors;
  if (observations.empty()) {
    return descriptors;
  }

  const auto & d = observations[0].descriptor;
  descriptors.create(observations.size(), d.cols, d.type());

  for (size_t i = 0; i < observations.size(); ++i) {
    observations[i].descriptor.copyTo(descriptors.row(i));
  }
  return descriptors;
}

std::vector<Observation> Frame::get_observations(ObservationFilter filter_type) const
{
  if (filter_type == ObservationFilter::ALL) {
    return observations;
  }
  std::vector<Observation> valid_obs;
  valid_obs.reserve(observations.size());
  for (const auto & obs : observations) {
    if (
      (filter_type == ObservationFilter::WITH_LANDMARKS && obs.landmark_id != -1) ||
      (filter_type == ObservationFilter::WITHOUT_LANDMARKS && obs.landmark_id == -1)) {
      valid_obs.push_back(obs);
    }
  }
  return valid_obs;
}

void Frame::clear_observations() { observations.clear(); }

}  // namespace mono_vo
// Copyright (c) 2026 Oscar Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/tools/human_speed_limiter.hpp"
#include "nav2_mppi_controller/tools/human_speed_limit_law.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mppi
{

void HumanSpeedLimiter::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  ParametersHandler * parameters_handler)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_buffer_ = tf_buffer;

  auto getParam = parameters_handler->getParamGetter(name + ".HumanSpeedLimiter");
  getParam(enabled_, "enabled", false, ParameterType::Static);
  getParam(tracking_topic_, "tracking_topic", std::string("tracking"), ParameterType::Static);
  getParam(prediction_horizon_, "prediction_horizon", 2.0, ParameterType::Static);
  getParam(prediction_step_, "prediction_step", 0.25, ParameterType::Static);
  getParam(min_obstacle_speed_, "min_obstacle_speed", 0.3);
  getParam(track_timeout_, "track_timeout", 0.5);
  getParam(disc_radius_, "disc_radius", 0.6);
  getParam(slow_distance_, "slow_distance", 3.6);
  getParam(stop_distance_, "stop_distance", 1.2);
  getParam(resume_delay_, "resume_delay", 1.0);
  getParam(resume_same_path_distance_, "resume_same_path_distance", 2.5);
  getParam(resume_same_path_tolerance_, "resume_same_path_tolerance", 0.1);
  getParam(visualize_, "visualize", false);

  if (!enabled_) {
    return;
  }
  if (prediction_step_ <= 0.0 || !std::isfinite(prediction_horizon_) || prediction_horizon_ < 0.0 ||
    slow_distance_ <= stop_distance_ || stop_distance_ < 0.0)
  {
    throw std::runtime_error(
            "HumanSpeedLimiter: prediction_step must be > 0, prediction_horizon finite and >= 0, and "
            "slow_distance > stop_distance >= 0");
  }
  prediction_steps_ = static_cast<int>(std::floor(prediction_horizon_ / prediction_step_));
  tracks_sub_ = node->create_subscription<nav2_dynamic_msgs::msg::ObstacleArray>(
    tracking_topic_, rclcpp::SensorDataQoS(),
    [this](const nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(tracks_mutex_);
      tracks_ = msg;
    });
  limit_pub_ = node->create_publisher<std_msgs::msg::Float32>("human_speed_limit", 1);
  discs_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("human_speed_limit/discs", 1);
  RCLCPP_INFO(
    logger_, "Human speed limiter: slows from %.2f m and stops at %.2f m of path before a predicted position "
    "within %.2f m of the plan", slow_distance_, stop_distance_, disc_radius_);
}

void HumanSpeedLimiter::cleanup()
{
  tracks_sub_.reset();
  limit_pub_.reset();
  discs_pub_.reset();
  std::lock_guard<std::mutex> lock(tracks_mutex_);
  tracks_.reset();
}

void HumanSpeedLimiter::activate()
{
  if (limit_pub_) {
    limit_pub_->on_activate();
    discs_pub_->on_activate();
  }
}

void HumanSpeedLimiter::deactivate()
{
  if (limit_pub_) {
    limit_pub_->on_deactivate();
    discs_pub_->on_deactivate();
  }
}

std::optional<double> HumanSpeedLimiter::speedRatio(
  const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & global_plan,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (!enabled_) {
    return std::nullopt;
  }
  nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr tracks;
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    tracks = tracks_;
  }
  auto publish = [this](const std::optional<double> & value) {
      if (limit_pub_ && limit_pub_->is_activated()) {
        std_msgs::msg::Float32 msg;
        msg.data = value ? static_cast<float>(*value) : -1.0f;
        limit_pub_->publish(msg);
      }
      return value;
    };
  if (!tracks || tracks->obstacles.empty() || global_plan.poses.size() < 2) {
    return publish(std::nullopt);
  }
  const double age = std::max(
    0.0, (clock_->now() - rclcpp::Time(tracks->header.stamp, clock_->get_clock_type())).seconds());
  if (age > track_timeout_) {
    return publish(std::nullopt);
  }

  // Everything is brought into the frame of the plan
  const std::string & plan_frame = global_plan.header.frame_id;
  tf2::Transform tracks_to_plan, robot_to_plan;
  try {
    tf2::fromMsg(
      tf_buffer_->lookupTransform(plan_frame, tracks->header.frame_id, tf2::TimePointZero).transform,
      tracks_to_plan);
    tf2::fromMsg(
      tf_buffer_->lookupTransform(plan_frame, robot_pose.header.frame_id, tf2::TimePointZero).transform,
      robot_to_plan);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Human speed limiter: %s", ex.what());
    return publish(std::nullopt);
  }
  tf2::Transform robot_in_frame;
  tf2::fromMsg(robot_pose.pose, robot_in_frame);
  const tf2::Transform robot = robot_to_plan * robot_in_frame;
  const double robot_x = robot.getOrigin().x(), robot_y = robot.getOrigin().y();
  std::vector<human_speed_limit::Point> footprint_in_plan;
  footprint_in_plan.reserve(footprint.size());
  for (const auto & corner : footprint) {
    const tf2::Vector3 p = robot * tf2::Vector3(corner.x, corner.y, 0.0);
    footprint_in_plan.emplace_back(p.x(), p.y());
  }

  std::vector<human_speed_limit::Point> predicted;
  for (const auto & track : tracks->obstacles) {
    const tf2::Vector3 position = tracks_to_plan * tf2::Vector3(track.position.x, track.position.y, 0.0);
    const tf2::Vector3 velocity =
      tracks_to_plan.getBasis() * tf2::Vector3(track.velocity.x, track.velocity.y, 0.0);
    if (std::hypot(velocity.x(), velocity.y()) < min_obstacle_speed_) {
      continue;
    }
    const auto positions = human_speed_limit::predictedPositions(
      {position.x(), position.y()}, {velocity.x(), velocity.y()}, age, prediction_step_, prediction_steps_,
      footprint_in_plan, disc_radius_);
    predicted.insert(predicted.end(), positions.begin(), positions.end());
  }

  std::vector<human_speed_limit::Point> plan;
  plan.reserve(global_plan.poses.size());
  for (const auto & pose : global_plan.poses) {
    plan.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  const auto conflict =
    human_speed_limit::firstConflict(plan, {robot_x, robot_y}, predicted, disc_radius_, slow_distance_);
  const double ratio =
    conflict ? human_speed_limit::speedRatio(conflict->distance, slow_distance_, stop_distance_) : 1.0;
  if (was_visualizing_ && !visualize_ && discs_pub_ && discs_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    discs_pub_->publish(markers);
  }
  was_visualizing_ = visualize_;
  if (visualize_) {
    visualize(
      plan_frame, predicted, plan,
      conflict ? std::make_optional(std::make_pair(conflict->plan_index, conflict->predicted_index)) : std::nullopt,
      ratio);
  }
  if (!conflict) {
    return publish(std::nullopt);
  }
  if (ratio <= 0.0) {
    stopped_ = true;
    stop_called_at_ = clock_->now();
  }
  return publish(ratio);
}

void HumanSpeedLimiter::planReceived(const nav_msgs::msg::Path & plan, const nav_msgs::msg::Path & previous)
{
  if (!enabled_ || plan.header.frame_id != previous.header.frame_id) {
    return;
  }
  std::vector<human_speed_limit::Point> points, previous_points;
  for (const auto & pose : plan.poses) {
    points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  for (const auto & pose : previous.poses) {
    previous_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  if (human_speed_limit::planDeviation(points, previous_points, resume_same_path_distance_) >
    resume_same_path_tolerance_)
  {
    plan_moved_at_ = clock_->now();
  }
}

bool HumanSpeedLimiter::holding()
{
  if (!stopped_) {
    return false;
  }
  const rclcpp::Time now = clock_->now();
  if ((now - stop_called_at_).seconds() < resume_delay_ ||
    (plan_moved_at_.nanoseconds() > 0 && (now - plan_moved_at_).seconds() < resume_delay_))
  {
    return true;
  }
  stopped_ = false;
  return false;
}

void HumanSpeedLimiter::reset()
{
  stopped_ = false;
}

void HumanSpeedLimiter::visualize(
  const std::string & frame, const std::vector<std::pair<double, double>> & predicted,
  const std::vector<std::pair<double, double>> & plan, const std::optional<std::pair<size_t, size_t>> & conflict,
  double ratio)
{
  if (!discs_pub_ || !discs_pub_->is_activated()) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker discs;
  discs.header.frame_id = frame;
  discs.header.stamp = clock_->now();
  discs.ns = "predicted";
  discs.id = 0;
  discs.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  discs.action = visualization_msgs::msg::Marker::ADD;
  discs.pose.orientation.w = 1.0;
  discs.scale.x = discs.scale.y = 2.0 * disc_radius_;
  discs.scale.z = 0.02;
  for (size_t k = 0; k < predicted.size(); ++k) {
    geometry_msgs::msg::Point point;
    point.x = predicted[k].first;
    point.y = predicted[k].second;
    discs.points.push_back(point);
    std_msgs::msg::ColorRGBA color;
    const bool in_conflict = conflict && conflict->second == k;
    color.r = 1.0f;
    color.g = in_conflict ? 0.0f : 0.8f;
    color.a = in_conflict ? 0.8f : 0.25f;
    discs.colors.push_back(color);
  }
  if (!discs.points.empty()) {
    markers.markers.push_back(discs);
  }

  if (conflict) {
    visualization_msgs::msg::Marker point;
    point.header = discs.header;
    point.ns = "conflict";
    point.id = 0;
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;
    point.pose.position.x = plan[conflict->first].first;
    point.pose.position.y = plan[conflict->first].second;
    point.pose.orientation.w = 1.0;
    point.scale.x = point.scale.y = point.scale.z = 0.2;
    point.color.r = 1.0f;
    point.color.a = 1.0f;
    markers.markers.push_back(point);

    visualization_msgs::msg::Marker text = point;
    text.ns = "limit";
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position.z = 0.6;
    text.scale.z = 0.25;
    text.color.g = text.color.b = 1.0f;
    text.text = ratio <= 0.0 ? "stop" : std::to_string(static_cast<int>(std::lround(100.0 * ratio))) + " %";
    markers.markers.push_back(text);
  }
  discs_pub_->publish(markers);
}

}  // namespace mppi

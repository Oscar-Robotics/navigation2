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
#ifndef NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMIT_LAW_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMIT_LAW_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace mppi
{
namespace human_speed_limit
{

using Point = std::pair<double, double>;

/**
 * @brief Distance from a point to the segment a-b.
 */
inline double distanceToSegment(const Point & point, const Point & a, const Point & b)
{
  const double seg_x = b.first - a.first, seg_y = b.second - a.second;
  const double length_sq = seg_x * seg_x + seg_y * seg_y;
  double u = 0.0;
  if (length_sq > 0.0) {
    u = std::clamp(((point.first - a.first) * seg_x + (point.second - a.second) * seg_y) / length_sq, 0.0, 1.0);
  }
  return std::hypot(point.first - (a.first + u * seg_x), point.second - (a.second + u * seg_y));
}

/**
 * @brief Predicted positions of an obstacle moving at constant velocity, up to where it reaches the robot.
 *
 * Positions are taken every step, from first_time, steps + 1 times. Once the obstacle's predicted course
 * comes within radius of the robot, that position and the later ones are left out: an obstacle that would
 * have reached the robot, whether it then stops or passes it, is not predicted beyond it. An obstacle
 * already within radius of the robot keeps its first position only.
 */
inline std::vector<Point> predictedPositions(
  const Point & position, const Point & velocity, double first_time, double step, int steps,
  const Point & robot, double radius)
{
  std::vector<Point> predicted;
  Point previous;
  for (int k = 0; k <= steps; ++k) {
    const double time = first_time + k * step;
    const Point current{position.first + velocity.first * time, position.second + velocity.second * time};
    if (k == 0) {
      predicted.push_back(current);
      if (std::hypot(current.first - robot.first, current.second - robot.second) <= radius) {
        break;
      }
    } else {
      if (distanceToSegment(robot, previous, current) <= radius) {
        break;
      }
      predicted.push_back(current);
    }
    previous = current;
  }
  return predicted;
}

struct Conflict
{
  /// Distance along the plan from the robot to the conflict point
  double distance;
  /// Index of the conflict point in the plan
  size_t plan_index;
  /// Index of the predicted position that comes onto the plan there
  size_t predicted_index;
};

/**
 * @brief First point of the plan, walking from the robot, that lies within disc_radius of a predicted
 * position.
 *
 * The plan is walked from its point closest to the robot, over at most max_distance.
 * @return The conflict, or nothing when no predicted position comes onto that stretch of the plan
 */
inline std::optional<Conflict> firstConflict(
  const std::vector<Point> & plan, const Point & robot, const std::vector<Point> & predicted,
  double disc_radius, double max_distance)
{
  if (plan.empty() || predicted.empty()) {
    return std::nullopt;
  }
  size_t start = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    const double d = std::hypot(plan[i].first - robot.first, plan[i].second - robot.second);
    if (d < best) {
      best = d;
      start = i;
    }
  }
  const double radius_sq = disc_radius * disc_radius;
  double travelled = 0.0;
  for (size_t i = start; i < plan.size(); ++i) {
    if (i > start) {
      travelled += std::hypot(plan[i].first - plan[i - 1].first, plan[i].second - plan[i - 1].second);
    }
    if (travelled > max_distance) {
      break;
    }
    for (size_t k = 0; k < predicted.size(); ++k) {
      const double dx = plan[i].first - predicted[k].first, dy = plan[i].second - predicted[k].second;
      if (dx * dx + dy * dy <= radius_sq) {
        return Conflict{travelled, i, k};
      }
    }
  }
  return std::nullopt;
}

/**
 * @brief Distance along the plan, from the robot, to the first conflict; see firstConflict.
 */
inline std::optional<double> distanceToConflict(
  const std::vector<Point> & plan, const Point & robot, const std::vector<Point> & predicted,
  double disc_radius, double max_distance)
{
  const auto conflict = firstConflict(plan, robot, predicted, disc_radius, max_distance);
  return conflict ? std::optional<double>(conflict->distance) : std::nullopt;
}

/**
 * @brief Largest distance from the first stretch of a plan to another plan.
 *
 * The first plan is walked from its start over at most length; for each of its points the distance to the
 * closest point of the other plan is taken. Used to tell a plan that moved from one that was only re-sent.
 * @return The largest of those distances, or infinity when either plan is empty
 */
inline double planDeviation(const std::vector<Point> & plan, const std::vector<Point> & other, double length)
{
  if (plan.empty() || other.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double worst = 0.0, travelled = 0.0;
  for (size_t i = 0; i < plan.size(); ++i) {
    if (i > 0) {
      travelled += std::hypot(plan[i].first - plan[i - 1].first, plan[i].second - plan[i - 1].second);
    }
    if (travelled > length) {
      break;
    }
    double nearest = std::numeric_limits<double>::max();
    for (const auto & p : other) {
      nearest = std::min(nearest, std::hypot(plan[i].first - p.first, plan[i].second - p.second));
    }
    worst = std::max(worst, nearest);
  }
  return worst;
}

/**
 * @brief Share of the full speed allowed at a distance from the conflict: 1 at slow_distance and beyond,
 * falling linearly to 0 at stop_distance and below.
 */
inline double speedRatio(double distance, double slow_distance, double stop_distance)
{
  if (distance >= slow_distance) {
    return 1.0;
  }
  if (distance <= stop_distance) {
    return 0.0;
  }
  return (distance - stop_distance) / (slow_distance - stop_distance);
}

}  // namespace human_speed_limit
}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMIT_LAW_HPP_

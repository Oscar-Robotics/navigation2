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
 * @brief Whether a point lies inside a polygon given by its corners in order.
 */
inline bool insidePolygon(const Point & point, const std::vector<Point> & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point & a = polygon[i];
    const Point & b = polygon[j];
    if ((a.second > point.second) != (b.second > point.second) &&
      point.first < (b.first - a.first) * (point.second - a.second) / (b.second - a.second) + a.first)
    {
      inside = !inside;
    }
  }
  return inside;
}

/**
 * @brief Whether the segments a-b and c-d cross.
 */
inline bool segmentsCross(const Point & a, const Point & b, const Point & c, const Point & d)
{
  const auto side = [](const Point & p, const Point & q, const Point & r) {
      return (q.first - p.first) * (r.second - p.second) - (q.second - p.second) * (r.first - p.first);
    };
  const double d1 = side(c, d, a), d2 = side(c, d, b), d3 = side(a, b, c), d4 = side(a, b, d);
  return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

/**
 * @brief Distance from the segment a-b to a polygon given by its corners in order; 0 when they overlap.
 */
inline double distanceToPolygon(const Point & a, const Point & b, const std::vector<Point> & polygon)
{
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (insidePolygon(a, polygon) || insidePolygon(b, polygon)) {
    return 0.0;
  }
  double nearest = std::numeric_limits<double>::max();
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (segmentsCross(a, b, polygon[i], polygon[j])) {
      return 0.0;
    }
    nearest = std::min(
      {nearest, distanceToSegment(a, polygon[i], polygon[j]), distanceToSegment(b, polygon[i], polygon[j]),
        distanceToSegment(polygon[i], a, b)});
  }
  return nearest;
}

/**
 * @brief Predicted positions of an obstacle moving at constant velocity, up to where it reaches the robot.
 *
 * Positions are taken every step, from first_time, steps + 1 times. Once a disc of the given radius around
 * the obstacle's predicted course touches the robot's footprint, that position and the later ones are left
 * out: an obstacle that would have reached the robot, whether it then stops or passes it, is not predicted
 * beyond it. An obstacle whose disc already touches the footprint has no predicted position.
 * @param footprint Corners of the robot's footprint, in order, in the frame of the positions
 */
inline std::vector<Point> predictedPositions(
  const Point & position, const Point & velocity, double first_time, double step, int steps,
  const std::vector<Point> & footprint, double radius)
{
  std::vector<Point> predicted;
  Point previous;
  for (int k = 0; k <= steps; ++k) {
    const double time = first_time + k * step;
    const Point current{position.first + velocity.first * time, position.second + velocity.second * time};
    if (distanceToPolygon(k == 0 ? current : previous, current, footprint) <= radius) {
      break;
    }
    predicted.push_back(current);
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

/**
 * @brief Share of the full speed allowed for a conflict at a distance along the plan with a position
 * predicted at a time in the future.
 *
 * The larger of two shares: the one from the distance (see above) and the one from the time, 0 at stop_time
 * and sooner, rising linearly to 1 at slow_time. The robot therefore stops only for a conflict that is both
 * within stop_distance and predicted within stop_time; a later one at the same place only slows it.
 */
inline double speedRatio(
  double distance, double time, double slow_distance, double stop_distance, double slow_time, double stop_time)
{
  return std::max(speedRatio(distance, slow_distance, stop_distance), speedRatio(time, slow_time, stop_time));
}

struct Limit
{
  /// Share of the full speed allowed
  double ratio;
  Conflict conflict;
};

/**
 * @brief Lowest share of the full speed over all predicted positions that come onto the plan.
 *
 * Each predicted position is taken with its own conflict point on the plan and its own time.
 * @param times Time in the future of each predicted position
 * @return The limit and the conflict that sets it, or nothing when no predicted position comes onto the plan
 */
inline std::optional<Limit> speedLimit(
  const std::vector<Point> & plan, const Point & robot, const std::vector<Point> & predicted,
  const std::vector<double> & times, double disc_radius, double slow_distance, double stop_distance,
  double slow_time, double stop_time)
{
  std::optional<Limit> limit;
  for (size_t k = 0; k < predicted.size() && k < times.size(); ++k) {
    auto conflict = firstConflict(plan, robot, {predicted[k]}, disc_radius, slow_distance);
    if (!conflict) {
      continue;
    }
    conflict->predicted_index = k;
    const double ratio =
      speedRatio(conflict->distance, times[k], slow_distance, stop_distance, slow_time, stop_time);
    if (!limit || ratio < limit->ratio) {
      limit = Limit{ratio, *conflict};
    }
  }
  return limit;
}

}  // namespace human_speed_limit
}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMIT_LAW_HPP_

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

#include <vector>

#include "gtest/gtest.h"
#include "nav2_mppi_controller/tools/human_speed_limit_law.hpp"

using mppi::human_speed_limit::distanceToConflict;
using mppi::human_speed_limit::planDeviation;
using mppi::human_speed_limit::Point;
using mppi::human_speed_limit::predictedPositions;
using mppi::human_speed_limit::speedLimit;
using mppi::human_speed_limit::speedRatio;

namespace
{
std::vector<Point> footprint()
{
  return {{0.33, 0.26}, {0.33, -0.26}, {-0.26, -0.26}, {-0.26, 0.26}};
}

std::vector<Point> straightPlan()
{
  std::vector<Point> plan;
  for (int i = 0; i <= 400; ++i) {
    plan.emplace_back(0.05 * i, 0.0);
  }
  return plan;
}
}  // namespace

TEST(HumanSpeedLimitLaw, distance_is_measured_along_the_plan_from_the_robot)
{
  const auto distance = distanceToConflict(straightPlan(), {2.0, 0.1}, {{6.0, 0.0}}, 0.6, 10.0);
  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, 3.4, 0.06);
}

TEST(HumanSpeedLimitLaw, a_prediction_beside_the_plan_is_not_a_conflict)
{
  EXPECT_FALSE(distanceToConflict(straightPlan(), {2.0, 0.0}, {{6.0, 0.7}}, 0.6, 10.0).has_value());
  EXPECT_TRUE(distanceToConflict(straightPlan(), {2.0, 0.0}, {{6.0, 0.5}}, 0.6, 10.0).has_value());
}

TEST(HumanSpeedLimitLaw, a_prediction_beyond_the_examined_distance_or_behind_the_robot_is_ignored)
{
  EXPECT_FALSE(distanceToConflict(straightPlan(), {2.0, 0.0}, {{9.0, 0.0}}, 0.6, 3.6).has_value());
  EXPECT_FALSE(distanceToConflict(straightPlan(), {8.0, 0.0}, {{3.0, 0.0}}, 0.6, 3.6).has_value());
}

TEST(HumanSpeedLimitLaw, the_nearest_conflict_along_the_plan_decides)
{
  const auto distance =
    distanceToConflict(straightPlan(), {0.0, 0.0}, {{5.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}}, 0.6, 10.0);
  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, 2.4, 0.06);
}

TEST(HumanSpeedLimitLaw, the_distance_follows_a_plan_that_turns)
{
  std::vector<Point> plan;
  for (int i = 0; i <= 40; ++i) {
    plan.emplace_back(0.05 * i, 0.0);
  }
  for (int i = 1; i <= 80; ++i) {
    plan.emplace_back(2.0, 0.05 * i);
  }
  const auto distance = distanceToConflict(plan, {0.0, 0.0}, {{2.0, 3.0}}, 0.3, 10.0);
  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, 4.7, 0.06);
}

TEST(HumanSpeedLimitLaw, speed_falls_linearly_from_the_slow_distance_to_a_stop)
{
  EXPECT_DOUBLE_EQ(speedRatio(5.0, 3.6, 1.2), 1.0);
  EXPECT_DOUBLE_EQ(speedRatio(3.6, 3.6, 1.2), 1.0);
  EXPECT_NEAR(speedRatio(2.4, 3.6, 1.2), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(speedRatio(1.2, 3.6, 1.2), 0.0);
  EXPECT_DOUBLE_EQ(speedRatio(0.3, 3.6, 1.2), 0.0);
}

TEST(HumanSpeedLimitLaw, nothing_is_found_on_an_empty_plan_or_past_its_end)
{
  EXPECT_FALSE(distanceToConflict({}, {0.0, 0.0}, {{1.0, 0.0}}, 0.6, 5.0).has_value());
  EXPECT_FALSE(distanceToConflict({{0.0, 0.0}}, {0.0, 0.0}, {{3.0, 0.0}}, 0.6, 5.0).has_value());
  EXPECT_FALSE(distanceToConflict(straightPlan(), {25.0, 0.0}, {{30.0, 0.0}}, 0.6, 5.0).has_value());
}

TEST(HumanSpeedLimitLaw, a_conflict_just_past_the_examined_distance_is_not_returned)
{
  EXPECT_FALSE(distanceToConflict(straightPlan(), {0.0, 0.0}, {{4.23, 0.0}}, 0.6, 3.6).has_value());
  EXPECT_TRUE(distanceToConflict(straightPlan(), {0.0, 0.0}, {{4.15, 0.0}}, 0.6, 3.6).has_value());
}

TEST(HumanSpeedLimitLaw, an_obstacle_coming_from_behind_is_not_predicted_past_the_robot)
{
  const auto overtaking = predictedPositions({-1.5, 0.2}, {1.5, 0.0}, 0.0, 0.25, 8, footprint(), 0.6);
  ASSERT_FALSE(overtaking.empty());
  for (const auto & p : overtaking) {
    EXPECT_LT(p.first, -0.5);
  }
  EXPECT_FALSE(distanceToConflict(straightPlan(), {0.0, 0.0}, overtaking, 0.6, 3.6).has_value());
}

TEST(HumanSpeedLimitLaw, an_obstacle_passing_wide_of_the_robot_is_predicted_ahead_of_it)
{
  const auto passing = predictedPositions({-1.5, 1.0}, {1.5, 0.0}, 0.0, 0.25, 8, footprint(), 0.6);
  EXPECT_EQ(passing.size(), 9u);
  EXPECT_GT(passing.back().first, 1.0);
}

TEST(HumanSpeedLimitLaw, an_oncoming_obstacle_is_predicted_up_to_the_robot_and_still_conflicts)
{
  const auto oncoming = predictedPositions({4.0, 0.0}, {-1.2, 0.0}, 0.0, 0.25, 16, footprint(), 0.6);
  ASSERT_FALSE(oncoming.empty());
  for (const auto & p : oncoming) {
    EXPECT_GT(p.first, 0.6);
  }
  const auto distance = distanceToConflict(straightPlan(), {0.0, 0.0}, oncoming, 0.6, 3.6);
  ASSERT_TRUE(distance.has_value());
  EXPECT_LT(*distance, 1.0);
}

TEST(HumanSpeedLimitLaw, a_fast_obstacle_cannot_step_over_the_robot_between_two_predictions)
{
  const auto fast = predictedPositions({-0.9, 0.5}, {4.0, 0.0}, 0.0, 0.5, 4, footprint(), 0.6);
  EXPECT_EQ(fast.size(), 1u);
}

TEST(HumanSpeedLimitLaw, an_obstacle_whose_disc_touches_the_footprint_has_no_predicted_position)
{
  EXPECT_TRUE(predictedPositions({-0.8, 0.0}, {0.6, 0.0}, 0.0, 0.25, 8, footprint(), 0.6).empty());
  EXPECT_TRUE(predictedPositions({0.9, 0.0}, {-0.6, 0.0}, 0.0, 0.25, 8, footprint(), 0.6).empty());
}

TEST(HumanSpeedLimitLaw, a_follower_beside_the_robot_centre_is_not_predicted_onto_the_path_ahead)
{
  const auto follower = predictedPositions({-1.5, 0.8}, {1.2, -0.3}, 0.0, 0.25, 8, footprint(), 0.6);
  EXPECT_FALSE(distanceToConflict(straightPlan(), {0.0, 0.0}, follower, 0.6, 3.6).has_value());
}

TEST(HumanSpeedLimitLaw, a_course_through_the_footprint_is_at_no_distance_from_it)
{
  EXPECT_DOUBLE_EQ(mppi::human_speed_limit::distanceToPolygon({-1.0, 0.0}, {1.0, 0.0}, footprint()), 0.0);
  EXPECT_DOUBLE_EQ(mppi::human_speed_limit::distanceToPolygon({0.0, 0.0}, {0.1, 0.0}, footprint()), 0.0);
  EXPECT_NEAR(mppi::human_speed_limit::distanceToPolygon({-1.0, 1.0}, {1.0, 1.0}, footprint()), 0.74, 1e-9);
}

TEST(HumanSpeedLimitLaw, the_robot_stops_only_for_a_conflict_both_close_and_soon)
{
  EXPECT_DOUBLE_EQ(speedRatio(1.0, 1.0, 3.6, 1.2, 3.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(speedRatio(1.0, 2.0, 3.6, 1.2, 3.0, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(speedRatio(2.4, 0.0, 3.6, 1.2, 3.0, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(speedRatio(3.6, 0.0, 3.6, 1.2, 3.0, 1.0), 1.0);
}

TEST(HumanSpeedLimitLaw, the_limit_is_set_by_the_predicted_position_that_allows_the_least_speed)
{
  const std::vector<Point> predicted{{0.8, 0.0}, {3.0, 0.0}, {5.0, 3.0}};
  const auto late_and_close = speedLimit(straightPlan(), {0.0, 0.0}, predicted, {2.0, 0.0, 0.0}, 0.6, 3.6, 1.2, 3.0, 1.0);
  ASSERT_TRUE(late_and_close.has_value());
  EXPECT_DOUBLE_EQ(late_and_close->ratio, 0.5);
  EXPECT_EQ(late_and_close->conflict.predicted_index, 0u);
  const auto soon_and_close = speedLimit(straightPlan(), {0.0, 0.0}, predicted, {0.5, 0.0, 0.0}, 0.6, 3.6, 1.2, 3.0, 1.0);
  ASSERT_TRUE(soon_and_close.has_value());
  EXPECT_DOUBLE_EQ(soon_and_close->ratio, 0.0);
  EXPECT_FALSE(speedLimit(straightPlan(), {0.0, 0.0}, {{5.0, 3.0}}, {0.0}, 0.6, 3.6, 1.2, 3.0, 1.0).has_value());
}

TEST(HumanSpeedLimitLaw, a_plan_sent_again_from_further_along_has_not_moved)
{
  const auto previous = straightPlan();
  const std::vector<Point> again(previous.begin() + 30, previous.end());
  EXPECT_NEAR(planDeviation(again, previous, 3.6), 0.0, 1e-9);
}

TEST(HumanSpeedLimitLaw, a_plan_that_swerves_within_the_examined_stretch_has_moved)
{
  auto swerving = straightPlan();
  for (size_t i = 20; i < 60; ++i) {
    swerving[i].second = 0.5;
  }
  EXPECT_NEAR(planDeviation(swerving, straightPlan(), 3.6), 0.5, 0.03);
  auto later = straightPlan();
  for (size_t i = 200; i < 240; ++i) {
    later[i].second = 0.5;
  }
  EXPECT_NEAR(planDeviation(later, straightPlan(), 3.6), 0.0, 1e-9);
}

TEST(HumanSpeedLimitLaw, a_first_plan_counts_as_moved)
{
  EXPECT_GT(planDeviation(straightPlan(), {}, 3.6), 1.0);
}

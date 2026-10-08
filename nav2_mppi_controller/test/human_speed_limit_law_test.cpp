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
using mppi::human_speed_limit::Point;
using mppi::human_speed_limit::predictedPositions;
using mppi::human_speed_limit::speedRatio;

namespace
{
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
  const auto overtaking = predictedPositions({-1.5, 0.2}, {1.5, 0.0}, 0.0, 0.25, 8, {0.0, 0.0}, 0.6);
  ASSERT_FALSE(overtaking.empty());
  for (const auto & p : overtaking) {
    EXPECT_LT(p.first, -0.5);
  }
  EXPECT_FALSE(distanceToConflict(straightPlan(), {0.0, 0.0}, overtaking, 0.6, 3.6).has_value());
}

TEST(HumanSpeedLimitLaw, an_obstacle_passing_wide_of_the_robot_is_predicted_ahead_of_it)
{
  const auto passing = predictedPositions({-1.5, 1.0}, {1.5, 0.0}, 0.0, 0.25, 8, {0.0, 0.0}, 0.6);
  EXPECT_EQ(passing.size(), 9u);
  EXPECT_GT(passing.back().first, 1.0);
}

TEST(HumanSpeedLimitLaw, an_oncoming_obstacle_is_predicted_up_to_the_robot_and_still_conflicts)
{
  const auto oncoming = predictedPositions({4.0, 0.0}, {-1.2, 0.0}, 0.0, 0.25, 16, {0.0, 0.0}, 0.6);
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
  const auto fast = predictedPositions({-0.9, 0.5}, {4.0, 0.0}, 0.0, 0.5, 4, {0.0, 0.0}, 0.6);
  EXPECT_EQ(fast.size(), 1u);
}

TEST(HumanSpeedLimitLaw, an_obstacle_already_at_the_robot_keeps_its_current_position_only)
{
  const auto touching = predictedPositions({0.4, 0.0}, {0.5, 0.0}, 0.0, 0.25, 8, {0.0, 0.0}, 0.6);
  ASSERT_EQ(touching.size(), 1u);
  const auto distance = distanceToConflict(straightPlan(), {0.0, 0.0}, touching, 0.6, 3.6);
  ASSERT_TRUE(distance.has_value());
  EXPECT_DOUBLE_EQ(*distance, 0.0);
}

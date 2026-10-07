// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    * @param model_delay_vx input delay on the vx axis
    * @param model_delay_vy input delay on the vy axis
    * @param model_delay_wz input delay on the wz axis
    */
  void initialize(
    const models::ControlConstraints & control_constraints, float model_dt,
    float model_delay_vx = 0.0f, float model_delay_vy = 0.0f, float model_delay_wz = 0.0f)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
    model_delay_vx_ = model_delay_vx;
    model_delay_vy_ = model_delay_vy;
    model_delay_wz_ = model_delay_wz;

    cmd_history_vx_.resize(offsetSteps(model_delay_vx_), 0.0f);
    cmd_history_vy_.resize(offsetSteps(model_delay_vy_), 0.0f);
    cmd_history_wz_.resize(offsetSteps(model_delay_wz_), 0.0f);
  }

  /**
    * @brief Push the most recently published command to the per-axis history
    *        ring buffers. Called once per controller cycle from the optimizer.
    */
  void pushCommandHistory(float vx, float vy, float wz)
  {
    pushOne(cmd_history_vx_, vx);
    pushOne(cmd_history_vy_, vy);
    pushOne(cmd_history_wz_, wz);
  }

  /**
    * @brief Zero the ring buffers
    */
  void clearCommandHistory()
  {
    std::fill(cmd_history_vx_.begin(), cmd_history_vx_.end(), 0.0f);
    std::fill(cmd_history_vy_.begin(), cmd_history_vy_.end(), 0.0f);
    std::fill(cmd_history_wz_.begin(), cmd_history_wz_.end(), 0.0f);
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    const bool is_holo = isHolonomic();
    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    float max_delta_wz = model_dt_ * control_constraints_.az_max;
    for (unsigned int i = 0; i != state.vx.shape(0); i++) {
      float vx_last = state.vx(i, 0);
      float vy_last = state.vy(i, 0);
      float wz_last = state.wz(i, 0);
      for (unsigned int j = 1; j != state.vx.shape(1); j++) {
        float & cvx_curr = state.cvx(i, j - 1);
        cvx_curr = std::clamp(cvx_curr, vx_last + min_delta_vx, vx_last + max_delta_vx);
        state.vx(i, j) = cvx_curr;
        vx_last = cvx_curr;

        float & cwz_curr = state.cwz(i, j - 1);
        cwz_curr = std::clamp(cwz_curr, wz_last - max_delta_wz, wz_last + max_delta_wz);
        state.wz(i, j) = cwz_curr;
        wz_last = cwz_curr;

        if (is_holo) {
          float & cvy_curr = state.cvy(i, j - 1);
          cvy_curr = std::clamp(cvy_curr, vy_last - max_delta_vy, vy_last + max_delta_vy);
          state.vy(i, j) = cvy_curr;
          vy_last = cvy_curr;
        }
      }
    }

    const unsigned int offset_vx = cmd_history_vx_.size();
    const unsigned int offset_vy = cmd_history_vy_.size();
    const unsigned int offset_wz = cmd_history_wz_.size();

    if (offset_vx > 0u || offset_wz > 0u || (is_holo && offset_vy > 0u)) {
      applyDelayShift(state, is_holo, offset_vx, offset_vy, offset_wz);
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  /**
    * @brief Apply the per-axis input-delay shift to velocity rollout.
    *
    * For j in [1, offset) — the delay window — fill `dst` with history[j]
    */
  void applyDelayShift(
    models::State & state, bool is_holo,
    unsigned int offset_vx, unsigned int offset_vy, unsigned int offset_wz) const
  {
    auto shift = [](xt::xtensor<float, 2> & velocities, unsigned int offset,
        const std::vector<float> & history) {
        const unsigned int rows = static_cast<unsigned int>(velocities.shape(0));
        const unsigned int cols = static_cast<unsigned int>(velocities.shape(1));
        if (offset == 0u || cols == 0u) {
          return;
        }

        const unsigned int shifted = (offset < cols) ? cols - offset : 0u;
        const unsigned int end = std::min(offset, cols);
        for (unsigned int i = 0; i != rows; i++) {
          // Shift cols in-place right-to-left by offset
          for (unsigned int k = shifted; k > 0; --k) {
            velocities(i, offset + k - 1) = velocities(i, k);
          }

          // Fill delay window cols with the in-flight commands from history.
          for (unsigned int j = 1; j < end; ++j) {
            velocities(i, j) = history[j];
          }
        }
      };

    shift(state.vx, offset_vx, cmd_history_vx_);
    shift(state.wz, offset_wz, cmd_history_wz_);

    if (is_holo) {
      shift(state.vy, offset_vy, cmd_history_vy_);
    }
  }

  /**
    * @brief Convert a delay in seconds to an offset in number of rollout steps, rounding to the nearest step.
    */
  std::size_t offsetSteps(float delay) const
  {
    if (delay <= 0.0f || model_dt_ <= 0.0f) {
      return 0u;
    }
    return static_cast<std::size_t>(std::floor(delay / model_dt_ + 0.5f));
  }

  /**
    * @brief Push a value to the back of a ring buffer and rotate the elements.
    */
  static void pushOne(std::vector<float> & buf, float v)
  {
    if (buf.empty()) {return;}
    std::rotate(buf.begin(), buf.begin() + 1, buf.end());
    buf.back() = v;
  }

  float model_dt_{0.0};
  float model_delay_vx_{0.0};
  float model_delay_vy_{0.0};
  float model_delay_wz_{0.0};

  // Per-axis ring buffer of recently published commands
  std::vector<float> cmd_history_vx_;
  std::vector<float> cmd_history_vy_;
  std::vector<float> cmd_history_wz_;

  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, (xt::fabs(vx) / xt::fabs(wz)) < min_turning_r_);
    view = xt::sign(wz) * xt::fabs(vx) / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

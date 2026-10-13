// Copyright (c) 2026 Oscar Robotics

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMITER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMITER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mppi
{

/**
 * @class mppi::HumanSpeedLimiter
 * @brief Speed limit from the predicted positions of tracked moving obstacles.
 *
 * A tracked moving obstacle is projected along its velocity over predict.time_horizon, up to where its
 * course comes within predict.prune_at_footprint_distance of the robot's footprint. A predicted position within
 * predict.disc_radius of the global plan is a conflict. Its distance along the plan and the time at which it is
 * predicted each give a share of the full speed, 1 at slow and 0 at stop (speed_ratio.global_path_distance,
 * speed_ratio.prediction_time); the larger share applies, so the robot stops only for a conflict that is both
 * close and soon. After a stop the robot holds for resume_cooldown.
 */
class HumanSpeedLimiter
{
  public:
    void initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string &name,
                    std::shared_ptr<tf2_ros::Buffer> tf_buffer, ParametersHandler *parameters_handler);

    void cleanup();
    void activate();
    void deactivate();

    /**
     * @brief Share of the full speed allowed in this control cycle
     * @param robot_pose Robot pose in the costmap frame
     * @param global_plan Plan as received by the controller, in its own frame
     * @param footprint Corners of the robot's footprint in the robot's frame
     * @return Ratio in [0, 1], 0 meaning stop, or nothing when no tracked obstacle calls for a limit
     */
    std::optional<double> speedRatio(const geometry_msgs::msg::PoseStamped &robot_pose,
                                     const nav_msgs::msg::Path &global_plan,
                                     const std::vector<geometry_msgs::msg::Point> &footprint);

    /**
     * @brief Tell the limiter a plan was received, so it can tell when the plan last moved
     * @param plan New plan
     * @param previous Plan it replaces
     */
    void planReceived(const nav_msgs::msg::Path &plan, const nav_msgs::msg::Path &previous);

    /**
     * @brief Whether the robot must stay stopped: a stop was called for and, since then, the stop condition
     * has not been clear, or the first resume_same_path_distance of the plan has not stayed the same
     * (within resume_same_path_tolerance), for resume_delay
     */
    bool holding();

    void reset();

  protected:
    /**
     * @brief Publish the predicted discs, the one in conflict in red, and the conflict point on the plan
     */
    void visualize(const std::string &frame, const std::vector<std::pair<double, double>> &predicted,
                   const std::vector<std::pair<double, double>> &plan,
                   const std::optional<std::pair<size_t, size_t>> &conflict, double ratio);

    rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
    rclcpp::Clock::SharedPtr clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr tracks_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr limit_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr discs_pub_;
    std::mutex tracks_mutex_;
    nav2_dynamic_msgs::msg::ObstacleArray::ConstSharedPtr tracks_;

    bool enabled_{false};
    bool visualize_{false};
    bool was_visualizing_{false};
    std::string tracking_topic_;
    double prediction_time_horizon_{0.0};
    double prediction_time_step_{0.0};
    int prediction_steps_{0};
    double min_obstacle_speed_{0.0};
    double tracks_max_age_{0.0};
    double disc_radius_{0.0};
    double prune_at_footprint_distance_{0.0};
    double global_path_distance_slow_{0.0};
    double global_path_distance_stop_{0.0};
    double prediction_time_slow_{0.0};
    double prediction_time_stop_{0.0};
    double resume_delay_{0.0};
    double resume_stable_path_length_{0.0};
    double resume_stable_path_tolerance_{0.0};
    bool stopped_{false};
    rclcpp::Time stop_called_at_{0, 0, RCL_ROS_TIME};
    rclcpp::Time plan_moved_at_{0, 0, RCL_ROS_TIME};
};

} // namespace mppi

#endif // NAV2_MPPI_CONTROLLER__TOOLS__HUMAN_SPEED_LIMITER_HPP_

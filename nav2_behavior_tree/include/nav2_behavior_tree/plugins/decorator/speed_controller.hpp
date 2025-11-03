// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child every at a rate proportional to
 * the speed of the robot. If the robot travels faster, this node will tick its child at a
 * higher frequency and reduce the tick frequency if the robot slows down
 * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
 *       It will re-initialize when halted.
 */
class SpeedController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpeedController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SpeedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
      BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
      BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
      BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
      BT::InputPort<nav_msgs::msg::Goals>(
        "goals", "Vector of navigation goals"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Navigation goal"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Calculate euclid distance of 2 pose in 2D map
   * @return float euclid distance value
   */

  inline bool canSeeGoal(float x1, float y1, float yaw, float x2, float y2, float threshold = M_PI){
    float dx = x2 - x1;
    float dy = y2 - y1;
    float goal_heading = std::atan2(dy, dx);
    
    float diff = goal_heading - yaw;
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    return std::fabs(diff) <= (threshold / 2.0f);
  }

  /**
   * @brief Scale the rate based speed
   * @return double Rate scaled by speed limits and clamped
   */
  inline double getScaledRate(const double & speed)
  {
    return std::max(
      std::min(
        (((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_,
        max_rate_), min_rate_);
  }

  /**
   * @brief Update period based on current smoothed speed and reset timer
   */
  inline void updatePeriod()
  {
    if (start_pose_process_ == true && release_ == false){
      if (period_ >= 10.0){
        //RCLCPP_INFO(node_->get_logger(), "DEBUG: Haven't release yet but timeout, force release now!");
        release_ = true;
        start_pose_process_ = false;
        period_ = 0.0;
        return;
      }
      //RCLCPP_INFO(node_->get_logger(), "DEBUG: Haven't release yet, increase period to 0.25s");
      period_ += 0.25;
    }
    else if (start_pose_process_ == true && release_ == true){
      //RCLCPP_INFO(node_->get_logger(), "DEBUG: Released, start replanning now");
      start_pose_process_ = false;
      //Request update period immediately
      period_ = 0.0;
    }
    else{
    auto velocity = odom_smoother_->getTwist();
    double speed = std::hypot(velocity.linear.x, velocity.linear.y);
    double rate = getScaledRate(speed);
    period_ = 1.0 / rate;
    }
  }

  void onTimer();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::TimerBase::SharedPtr timer_;
  // To keep track of time to reset
  rclcpp::Time start_;

  // To get a smoothed velocity
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  bool first_tick_;
  bool start_pose_process_;
  bool release_;

  // Time period after which child node should be ticked
  double period_;

  // Rates thresholds to tick child node
  double min_rate_;
  double max_rate_;
  double d_rate_;

  // Speed thresholds
  double min_speed_;
  double max_speed_;
  double d_speed_;

  // Current position
  double current_x_;
  double current_y_;
  double current_z_;

  double transform_tolerance_;

  // current goal
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Goals goals_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SPEED_CONTROLLER_HPP_

// Copyright (c) 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/node_signal.hpp"
#include "nav2_msgs/msg/warning_command.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/service_client.hpp"


namespace nav2_behavior_tree
{

using NodeSignal = nav2_msgs::msg::NodeSignal;
using WarningCommand = nav2_msgs::msg::WarningCommand;
/**
 * @brief A BT::DecoratorNode that ticks its child every time when the length of
 * the new path is smaller than the old one by the length given by the user.
 */
class PathLongerOnApproach : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PathLongerOnApproach
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PathLongerOnApproach(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<nav_msgs::msg::Path>();

    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "prox_len", 3.0,
        "Proximity length (m) for the path to be longer on approach"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "Length multiplication factor to check if the path is significantly longer"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    };
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief Checks if the global path is updated
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @return whether the path is updated for the current goal
   */
  bool isPathUpdated(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path);

  /**
   * @brief Checks if the robot is in the goal proximity
   * @param old_path current path to the goal
   * @param prox_leng proximity length from the goal
   * @return whether the robot is in the goal proximity
   */
  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & old_path,
    double & prox_leng);

  /**
   * @brief Checks if the new path is longer
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @param length_factor multiplier for path length check
   * @return whether the new path is longer
   */
  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    double & length_factor);

private:
  nav_msgs::msg::Path new_path_;
  nav_msgs::msg::Path old_path_;
  double prox_len_ = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<NodeSignal>::SharedPtr signal_pub_;
  rclcpp::Publisher<WarningCommand>::SharedPtr warning_cmd_pub_;
  bool first_time_ = true;
  bool current_signal_state_ = false;
  bool warning_cmd_sent_ = false;
  nav2_util::ServiceClient<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  // The timeout value while waiting for a response from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
  unsigned int max_cost_ = 253;
  bool consider_unknown_as_obstacle_ = false;
  bool is_current_path_valid_ = true;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

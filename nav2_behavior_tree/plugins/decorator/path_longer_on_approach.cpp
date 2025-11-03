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

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"
#include "rclcpp/qos.hpp"

namespace nav2_behavior_tree
{

PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::QoS node_signal_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
  client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::IsPathValid>>("is_path_valid",
      node_, false /* Does not create and spin an internal executor*/);
  signal_pub_ = rclcpp::create_publisher<NodeSignal>(node_, "/node_signal", node_signal_qos);
  warning_cmd_pub_ = rclcpp::create_publisher<WarningCommand>(node_, "/audio/warn/command", 10);
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
}

bool PathLongerOnApproach::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  return old_path.poses.size() != 0 &&
         new_path.poses.size() != 0 &&
         new_path.poses.size() != old_path.poses.size() &&
         old_path.poses.back().pose.position == new_path.poses.back().pose.position;
}

bool PathLongerOnApproach::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathLongerOnApproach::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
         length_factor * nav2_util::geometry_utils::calculate_path_length(
    old_path, 0);
}

inline BT::NodeStatus PathLongerOnApproach::tick()
{
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);

  if (first_time_ == false) {
    if (old_path_.poses.empty() || new_path_.poses.empty() ||
      old_path_.poses.back().pose != new_path_.poses.back().pose)
    {
      first_time_ = true;
    }
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (!old_path_.poses.empty())
  {
    auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

    request->path = old_path_;
    request->max_cost = max_cost_;
    request->consider_unknown_as_obstacle = consider_unknown_as_obstacle_;
    auto response = client_->invoke(request);

    is_current_path_valid_ = response->is_valid;
  }

  // Check if the path is updated and valid, compare the old and the new path length,
  // given the goal proximity and check if the new path is longer
  if (isPathUpdated(new_path_, old_path_) && isRobotInGoalProximity(old_path_, prox_len_) &&
    isNewPathLonger(new_path_, old_path_, length_factor_) && !first_time_ && !is_current_path_valid_)
  {
    //Publish stuck signal
    NodeSignal node_signal_msg;
    node_signal_msg.signal = node_signal_msg.STUCK;
    node_signal_msg.state = true;
    if (current_signal_state_ != node_signal_msg.state)
    {
      signal_pub_->publish(node_signal_msg);
      current_signal_state_ = node_signal_msg.state;
    }

    if (!warning_cmd_sent_)
    {
      WarningCommand warning_cmd_msg;
      warning_cmd_msg.cmd = WarningCommand::PLAY_SIGNAL;
      warning_cmd_msg.signal = WarningCommand::ROBOT_STUCK;
      warning_cmd_pub_->publish(warning_cmd_msg);
      warning_cmd_sent_ = true;
    }

    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        resetChild();
        return child_state;
      default:
        old_path_ = new_path_;
        return BT::NodeStatus::FAILURE;
    }
  }
  old_path_ = new_path_;
  first_time_ = false;

  //Publish not stuck signal
  NodeSignal node_signal_msg;
  node_signal_msg.signal = node_signal_msg.STUCK;
  node_signal_msg.state = false;
  if (current_signal_state_ != node_signal_msg.state)
  {
    signal_pub_->publish(node_signal_msg);
    current_signal_state_ = node_signal_msg.state;
  }
  warning_cmd_sent_ = false;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}

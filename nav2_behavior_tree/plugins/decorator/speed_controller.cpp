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

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/speed_controller.hpp"

namespace nav2_behavior_tree
{

SpeedController::SpeedController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_tick_(false),
  period_(1.0),
  min_rate_(0.1),
  max_rate_(1.0),
  min_speed_(0.0),
  max_speed_(0.5)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("min_rate", min_rate_);
  getInput("max_rate", max_rate_);
  getInput("min_speed", min_speed_);
  getInput("max_speed", max_speed_);

  if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
    std::string err_msg = "SpeedController node cannot have rate <= 0.0";
    RCLCPP_FATAL(node_->get_logger(), "%s", err_msg.c_str());
    throw BT::BehaviorTreeException(err_msg);
  }

  d_rate_ = max_rate_ - min_rate_;
  d_speed_ = max_speed_ - min_speed_;

  odom_smoother_ = config().blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>(
    "odom_smoother");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SpeedController::onTimer, this),
      callback_group_);
      
  start_pose_process_ = false;
  release_ = false;

  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));

}

void SpeedController::onTimer() {
  try {
    geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    current_x_ = t.transform.translation.x;
    current_y_ = t.transform.translation.y;
    geometry_msgs::msg::Quaternion q_msg = t.transform.rotation;
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_z_ = yaw;
    // RCLCPP_INFO(this->node_->get_logger(), "Current x: %f, Current y: %f, Current z: %f", this->current_x_, this->current_y_, this->current_z_);

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
  }
}

inline BT::NodeStatus SpeedController::tick()
{
  if (!BT::isStatusActive(status())) {
    // Reset since we're starting a new iteration of
    // the speed controller (moving from IDLE to RUNNING)
    BT::getInputOrBlackboard("goals", goals_);
    BT::getInputOrBlackboard("goal", goal_);
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
  }

  callback_group_executor_.spin_some();

  nav_msgs::msg::Goals current_goals;
  BT::getInputOrBlackboard("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  BT::getInputOrBlackboard("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    // Reset state and set period to max since we have a new goal
    period_ = 1.0 / max_rate_;
    start_ = node_->now();
    first_tick_ = true;
    goal_ = current_goal;
    goals_ = current_goals;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto elapsed = node_->now() - start_;

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    elapsed.seconds() >= period_)
  {

    if (first_tick_ == true){
      try {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        current_x_ = t.transform.translation.x;
        current_y_ = t.transform.translation.y;
        geometry_msgs::msg::Quaternion q_msg = t.transform.rotation;
        tf2::Quaternion q;
        tf2::fromMsg(q_msg, q);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_z_ = yaw;
        RCLCPP_INFO(this->node_->get_logger(), "Init x: %f, Init y: %f, Init z: %f", current_x_, current_y_, current_z_);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->node_->get_logger(), "TF lookup failed: %s", ex.what());
      }
      start_pose_process_ = true;
      release_ = false;
    }

    first_tick_ = false;
    
    // update period if the last period is exceeded
    if (elapsed.seconds() >= period_) {
      if (start_pose_process_ == true){
        if (!canSeeGoal(current_x_, current_y_, current_z_, goal_.pose.position.x, goal_.pose.position.y))
        {
          updatePeriod();
          //RCLCPP_INFO(node_->get_logger(), "DEBUG: Robot cant see goal, waiting more 0.25s before planning");
          return BT::NodeStatus::RUNNING;
        } 
        else
        {
          release_ = true;
          //RCLCPP_INFO(node_->get_logger(), "DEBUG: Robot can see goal, update period now");
        }
      }
      updatePeriod();
      start_ = node_->now();
    }

    return child_node_->executeTick();
  }

  return status();
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SpeedController>("SpeedController");
}

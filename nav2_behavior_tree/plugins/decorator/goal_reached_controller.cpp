#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/plugins/decorator/goal_reached_controller.hpp"

namespace nav2_behavior_tree
{

GoalReachedController::GoalReachedController(
  const std::string& name,
  const BT::NodeConfiguration& conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_   = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  robot_base_frame_ = BT::deconflictPortAndParamFrame<std::string>(
      node_, "robot_base_frame", this);

  getInput("tolerance", goal_reached_tol_);

  // Provide a sane default if param is missing
  transform_tolerance_ = 0.1;
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  first_tick_ = true;
  child_latched_ = false;
}

bool GoalReachedController::isGoalReached()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_DEBUG(node_->get_logger(), "No 'goal' on port.");
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, goal.header.frame_id, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  const double dx = goal.pose.position.x - current_pose.pose.position.x;
  const double dy = goal.pose.position.y - current_pose.pose.position.y;
  return (dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_);
}

BT::NodeStatus GoalReachedController::tick()
{
  // First activation of this node in the current run
  if (!BT::isStatusActive(status())) {
    child_latched_ = false;
    (void) getInput("goal", goal_);
  }

  const bool reached = isGoalReached();

  // If the child is not started yet (not latched)...
  if (!child_latched_) {
    // GATE CLOSED: not at goal -> report RUNNING so parents (e.g., Parallel) keep going
    if (!reached) {
      return BT::NodeStatus::RUNNING;
    }

    // GATE OPEN: at goal -> start child ONCE
    const BT::NodeStatus s = child_node_->executeTick();
    switch (s) {
      case BT::NodeStatus::RUNNING:
        child_latched_ = true;          // latch and keep ticking next time
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        resetChild();                   // prepare for next time
        child_latched_ = false;
        return s;
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  // CHILD LATCHED: keep ticking every cycle so Delay/Retry progress
  // Optional: if you want to CANCEL the countdown when you slip out of "reached", do:
  if (!reached) {
    // cancel and unlatch (so it restarts when you reach goal again)
    haltChild();
    child_latched_ = false;
    return BT::NodeStatus::RUNNING;
  }

  const BT::NodeStatus s = child_node_->executeTick();  // keep ticking!
  if (s == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  // Terminal: propagate and reset
  resetChild();
  child_latched_ = false;
  return s;
}

void GoalReachedController::halt()
{
  if (child_latched_) {
    haltChild();
  }
  child_latched_ = false;
  BT::DecoratorNode::halt();
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalReachedController>("GoalReachedController");
}

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_REACHED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_REACHED_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class GoalReachedController : public BT::DecoratorNode
{
public:
  GoalReachedController(const std::string& name, const BT::NodeConfiguration& conf);

  BT::NodeStatus tick() override;
  
  void halt() override; // NEW

  bool isGoalReached();

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
      BT::InputPort<double>("tolerance", 0.25, "Goal reached tolerance")
      // If you actually need it, also declare: BT::InputPort<nav_msgs::msg::Goals>("goals")
    };
  }

protected:
  void cleanup() {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Time start_;

  // BOOK-KEEPING
  bool child_latched_ = false;   // NEW: child was started and is running
  bool first_tick_ = true;
  double goal_reached_tol_ = 0.25;
  double transform_tolerance_ = 0.1;
  std::string robot_base_frame_;

  geometry_msgs::msg::PoseStamped goal_;
  // nav_msgs::msg::Goals goals_; // only if you really use it
};

}  // namespace nav2_behavior_tree

#endif

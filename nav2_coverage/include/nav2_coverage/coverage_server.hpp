// Copyright ...
#ifndef NAV2_COVERAGE__COVERAGE_SERVER_HPP_
#define NAV2_COVERAGE__COVERAGE_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/cover_all_map.hpp"

#include "nav2_coverage/poses_creator.hpp"

namespace nav2_coverage
{

class CoverageServer : public nav2_util::LifecycleNode
{
public:
  explicit CoverageServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CoverageServer() override = default;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  using CoverAllMap = nav2_msgs::action::CoverAllMap;
  using GoalHandleCoverAllMap = rclcpp_action::ServerGoalHandle<CoverAllMap>;

  using Compute = nav2_msgs::action::ComputePathThroughPoses;
  using Follow = nav2_msgs::action::FollowPath;

  // Subscriptions
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  // Latest data
  std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg_;

  // Publishers (debug)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr graph_nodes_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;

  // Action server
  rclcpp_action::Server<CoverAllMap>::SharedPtr cover_action_server_;
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CoverAllMap::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCoverAllMap> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleCoverAllMap> goal_handle);

  // ---- Worker thread + preemption ----
  void workerLoop();
  void runPipeline(const std::shared_ptr<GoalHandleCoverAllMap> goal_handle);

  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  std::shared_ptr<GoalHandleCoverAllMap> active_goal_;
  std::shared_ptr<GoalHandleCoverAllMap> pending_goal_;

  std::thread worker_thread_;
  std::atomic_bool stop_worker_{false};
  std::atomic_bool preempt_requested_{false};

  // Action clients
  rclcpp_action::Client<Compute>::SharedPtr compute_client_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;

  // Components
  std::unique_ptr<PosesCreator> poses_creator_;

  // Helpers
  static std::string toLower(std::string s);

  bool waitForMapCostmap(
    nav_msgs::msg::OccupancyGrid::SharedPtr & map,
    nav_msgs::msg::OccupancyGrid::SharedPtr & costmap,
    double timeout_sec,
    const std::shared_ptr<GoalHandleCoverAllMap> & goal_handle);

  nav_msgs::msg::Path downsamplePath(const nav_msgs::msg::Path & in) const;

  static double distXY(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);

  static builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time & t);

  // Parameters (cached)
  std::string map_topic_;
  std::string costmap_topic_;

  std::string graph_nodes_topic_;
  std::string debug_path_topic_;

  std::string cover_action_name_;
  std::string compute_action_name_;
  std::string follow_action_name_;

  bool set_start_from_first_pose_{false};
  std::string planner_id_;
  std::string controller_id_;
  std::string goal_checker_id_;

  // Timeouts
  double wait_grid_timeout_sec_{5.0};
  double compute_timeout_sec_{30.0};
  double follow_timeout_sec_{0.0};  // 0 => no timeout

  // Downsample computed path
  int downsample_keep_every_n_{0};
  double downsample_min_dist_{0.0};
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__COVERAGE_SERVER_HPP_

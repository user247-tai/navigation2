#include "nav2_coverage/coverage_server.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_coverage
{

namespace
{
template<typename GoalT>
auto setPlannerId(GoalT & goal, const std::string & planner_id, int)
  -> decltype(goal.planner_id = planner_id, void())
{
  goal.planner_id = planner_id;
}
template<typename GoalT>
void setPlannerId(GoalT &, const std::string &, long) {}

template<typename GoalT>
auto setComputeGoals(
  GoalT & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & list,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  int) -> decltype(goal.goals.goals = list, void())
{
  goal.goals.header.stamp = stamp;
  goal.goals.header.frame_id = frame_id;
  goal.goals.goals = list;
}
template<typename GoalT>
auto setComputeGoals(
  GoalT & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & list,
  const builtin_interfaces::msg::Time &,
  const std::string &,
  long) -> decltype(goal.goals = list, void())
{
  goal.goals = list;
}

template<typename GoalT>
auto setControllerId(GoalT & goal, const std::string & id, int)
  -> decltype(goal.controller_id = id, void())
{
  goal.controller_id = id;
}
template<typename GoalT>
void setControllerId(GoalT &, const std::string &, long) {}

template<typename GoalT>
auto setGoalCheckerId(GoalT & goal, const std::string & id, int)
  -> decltype(goal.goal_checker_id = id, void())
{
  goal.goal_checker_id = id;
}
template<typename GoalT>
void setGoalCheckerId(GoalT &, const std::string &, long) {}
}  // namespace

CoverageServer::CoverageServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("coverage_server", "", options), costmap_(nullptr), current_index_(0)
{
  // topics
  declare_parameter("map_topic", "/map");
  declare_parameter("costmap_topic", "/global_costmap/costmap");

  declare_parameter("graph_nodes_topic", "/graph_nodes");
  declare_parameter("debug_path_topic", "/debug/computed_path");

  // actions
  declare_parameter("cover_action_name", "/cover_all_map");
  declare_parameter("compute_action_name", "/compute_path_through_poses");
  declare_parameter("follow_action_name", "/follow_path");

  // behavior
  declare_parameter("set_start_from_first_pose", true);
  declare_parameter("planner_id", "");
  declare_parameter("controller_id", "");
  declare_parameter("goal_checker_id", "");
  declare_parameter("retries_on_failure", 0);
  declare_parameter("recovery_obstacle_max_cost", 100);

  // timeouts
  declare_parameter("wait_grid_timeout_sec", 5.0);
  declare_parameter("compute_timeout_sec", 30.0);
  declare_parameter("follow_timeout_sec", 0.0);  // 0 => no timeout

  // downsample computed path
  declare_parameter("downsample_keep_every_n", 0);
  declare_parameter("downsample_min_dist", 0.0);

  // ---- PosesCreator options ----
  declare_parameter("grid_step_x", 0.05);
  declare_parameter("grid_step_y", 0.05);
  declare_parameter("allow_unknown_map", false);
  declare_parameter("allow_unknown_costmap", false);
  declare_parameter("skip_outside_costmap", true);

  // Setup callback group
  coverage_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  update_pose_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()},
    get_parameter("use_sim_time").as_bool());
}

CoverageServer::~CoverageServer()
{
  // Ensure that the worker thread is stopped
  stop_worker_.store(true);
  goal_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  costmap_thread_.reset();
}

builtin_interfaces::msg::Time CoverageServer::toTimeMsg(const rclcpp::Time & t)
{
  builtin_interfaces::msg::Time msg;
  const int64_t ns = t.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}

nav2_util::CallbackReturn CoverageServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  map_topic_ = get_parameter("map_topic").as_string();
  costmap_topic_ = get_parameter("costmap_topic").as_string();

  graph_nodes_topic_ = get_parameter("graph_nodes_topic").as_string();
  debug_path_topic_ = get_parameter("debug_path_topic").as_string();

  cover_action_name_ = get_parameter("cover_action_name").as_string();
  compute_action_name_ = get_parameter("compute_action_name").as_string();
  follow_action_name_ = get_parameter("follow_action_name").as_string();

  set_start_from_first_pose_ = get_parameter("set_start_from_first_pose").as_bool();
  planner_id_ = get_parameter("planner_id").as_string();
  controller_id_ = get_parameter("controller_id").as_string();
  goal_checker_id_ = get_parameter("goal_checker_id").as_string();
  retries_on_failure_ = get_parameter("retries_on_failure").as_int();
  recovery_obstacle_max_cost_ = get_parameter("recovery_obstacle_max_cost").as_int();

  wait_grid_timeout_sec_ = get_parameter("wait_grid_timeout_sec").as_double();
  compute_timeout_sec_ = get_parameter("compute_timeout_sec").as_double();
  follow_timeout_sec_ = get_parameter("follow_timeout_sec").as_double();

  downsample_keep_every_n_ = get_parameter("downsample_keep_every_n").as_int();
  downsample_min_dist_ = get_parameter("downsample_min_dist").as_double();

  // Create PosesCreator
  PosesCreator::Options pc;
  pc.grid_step_x = get_parameter("grid_step_x").as_double();
  pc.grid_step_y = get_parameter("grid_step_y").as_double();
  pc.allow_unknown_map = get_parameter("allow_unknown_map").as_bool();
  pc.allow_unknown_costmap = get_parameter("allow_unknown_costmap").as_bool();
  pc.skip_outside_costmap = get_parameter("skip_outside_costmap").as_bool();
  poses_creator_ = std::make_unique<PosesCreator>(pc, get_logger());

  // QoS for latched map/costmap
  rclcpp::QoS qos_grid(1);
  qos_grid.reliable();
  qos_grid.transient_local();

  // Subscription options
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = coverage_cb_group_;

  // Create subscriptions
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, qos_grid,
    std::bind(&CoverageServer::onMap, this, std::placeholders::_1), sub_opts);

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, qos_grid,
    std::bind(&CoverageServer::onCostmap, this, std::placeholders::_1), sub_opts);

  // Configure costmap
  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();
  if (!costmap_ros_->getUseRadius()) {
    collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  }
  
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  // Debug publishers (latched)
  rclcpp::QoS qos_debug(1);
  qos_debug.reliable();
  qos_debug.transient_local();

  graph_nodes_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(graph_nodes_topic_, qos_debug);
  debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(debug_path_topic_, qos_debug);

  // Action clients
  compute_client_ = rclcpp_action::create_client<Compute>(shared_from_this(), compute_action_name_);
  follow_client_ = rclcpp_action::create_client<Follow>(shared_from_this(), follow_action_name_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  graph_nodes_pub_->on_activate();
  debug_path_pub_->on_activate();

  // Create action server
  cover_action_server_ = rclcpp_action::create_server<CoverAllMap>(
    shared_from_this(),
    cover_action_name_,
    std::bind(&CoverageServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CoverageServer::handleCancel, this, std::placeholders::_1),
    std::bind(&CoverageServer::handleAccepted, this, std::placeholders::_1));

  // Start worker thread once
  stop_worker_.store(false);
  if (!worker_thread_.joinable()) {
    worker_thread_ = std::thread(&CoverageServer::workerLoop, this);
  }

  const auto costmap_ros_state = costmap_ros_->activate();
  if (costmap_ros_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  // Stop worker cleanly first
  stop_worker_.store(true);
  goal_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  cover_action_server_.reset();
  graph_nodes_pub_->on_deactivate();
  debug_path_pub_->on_deactivate();

  costmap_ros_->deactivate();

  current_index_ = 0;
  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cover_action_server_.reset();
  map_sub_.reset();
  costmap_sub_.reset();
  graph_nodes_pub_.reset();
  debug_path_pub_.reset();
  compute_client_.reset();
  follow_client_.reset();
  poses_creator_.reset();

  {
    std::lock_guard<std::mutex> lk(grid_mutex_);
    map_msg_.reset();
    costmap_msg_.reset();
  }

  costmap_ros_->cleanup();
  costmap_thread_.reset();
  costmap_ = nullptr;
  current_index_ = 0;

  RCLCPP_INFO(get_logger(), "Cleaned up coverage_server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void CoverageServer::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(grid_mutex_);
  map_msg_ = msg;
}

void CoverageServer::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(grid_mutex_);
  costmap_msg_ = msg;
}

void CoverageServer::updateRobotPose()
{
  std::lock_guard<std::mutex> lk(goal_list_mutex_);
  if (!costmap_ros_ || (goal_list_.empty())) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (costmap_ros_->getRobotPose(pose)) {
    size_t temp_index = findNearestIndex(goal_list_, pose);
    size_t index_offset = 0;
    
    if ((active_goal_ != nullptr) && (active_goal_->get_goal()->order_mode == "columns")) {
      index_offset = static_cast<size_t>(std::ceil(1.0 / static_cast<float>(active_goal_->get_goal()->downsample_step_y)));
    }
    else if (active_goal_ != nullptr) {
      index_offset = static_cast<size_t>(std::ceil(1.0 / static_cast<float>(active_goal_->get_goal()->downsample_step_x)));
    }

    if ((temp_index >= current_index_ + 1) && (temp_index <= current_index_ + index_offset)) {
      current_index_ = temp_index;
    }
  }
}

rclcpp_action::GoalResponse CoverageServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const CoverAllMap::Goal> goal)
{
  const std::string mode = toLower(goal->order_mode);
  if (mode != "rows" && mode != "columns") {
    RCLCPP_WARN(get_logger(),
      "CoverAllMap goal order_mode='%s' invalid, will treat as 'rows'",
      goal->order_mode.c_str());
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoverageServer::handleCancel(
  const std::shared_ptr<GoalHandleCoverAllMap>)
{
  RCLCPP_INFO(get_logger(), "Cancel requested for CoverAllMap");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoverageServer::handleAccepted(const std::shared_ptr<GoalHandleCoverAllMap> goal_handle)
{
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    pending_goal_ = goal_handle;
    if (active_goal_) {
      // Request preemption of current pipeline
      preempt_requested_.store(true);
    }
  }
  goal_cv_.notify_one();
}

void CoverageServer::workerLoop()
{
  while (rclcpp::ok() && !stop_worker_.load()) {
    std::shared_ptr<GoalHandleCoverAllMap> goal;

    {
      std::unique_lock<std::mutex> lk(goal_mutex_);
      goal_cv_.wait(lk, [&]() { return stop_worker_.load() || pending_goal_ != nullptr; });
      if (stop_worker_.load()) {
        return;
      }
      goal = pending_goal_;
      pending_goal_.reset();
      active_goal_ = goal;
      preempt_requested_.store(false);
    }

    runPipeline(goal);

    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      if (active_goal_ == goal) {
        active_goal_.reset();
      }
    }
  }
}

std::string CoverageServer::toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

size_t CoverageServer::findNearestIndex(const std::vector<geometry_msgs::msg::PoseStamped> & goals, const geometry_msgs::msg::PoseStamped & robot_pose)
{
  size_t best_i = 0;
  double best_d = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < goals.size(); ++i) {
    const double dx = goals[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = goals[i].pose.position.y - robot_pose.pose.position.y;
    const double d = std::hypot(dx, dy);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }

  size_t index_offset = 0;
  if ((active_goal_ != nullptr) && (active_goal_->get_goal()->order_mode == "columns")) {
    index_offset = static_cast<size_t>(std::ceil(1.0 / static_cast<float>(active_goal_->get_goal()->downsample_step_y)));
  }
  else if (active_goal_ != nullptr) {
    index_offset = static_cast<size_t>(std::ceil(1.0 / static_cast<float>(active_goal_->get_goal()->downsample_step_x)));
  }

  if ((best_i >= current_index_ + 1) && (best_i <= current_index_ + index_offset)) {
    return best_i;
  }
  return current_index_;
}

bool CoverageServer::waitForMapCostmap(
  nav_msgs::msg::OccupancyGrid::SharedPtr & map,
  nav_msgs::msg::OccupancyGrid::SharedPtr & costmap,
  double timeout_sec,
  const std::shared_ptr<GoalHandleCoverAllMap> & goal_handle)
{
  const auto start = now();
  rclcpp::Rate r(20.0);

  while (rclcpp::ok() && !stop_worker_.load()) {
    if (goal_handle && goal_handle->is_canceling()) {
      return false;
    }
    if (preempt_requested_.load()) {
      return false;
    }

    {
      std::lock_guard<std::mutex> lk(grid_mutex_);
      map = map_msg_;
      costmap = costmap_msg_;
    }

    if (map && costmap) {
      return true;
    }

    if (timeout_sec > 0.0 && (now() - start).seconds() > timeout_sec) {
      return false;
    }

    r.sleep();
  }
  return false;
}

double CoverageServer::distXY(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

nav_msgs::msg::Path CoverageServer::downsamplePath(const nav_msgs::msg::Path & in) const
{
  if (in.poses.empty()) return in;

  if (downsample_keep_every_n_ > 1) {
    nav_msgs::msg::Path out;
    out.header = in.header;
    out.poses.push_back(in.poses.front());
    for (size_t i = static_cast<size_t>(downsample_keep_every_n_);
         i < in.poses.size();
         i += static_cast<size_t>(downsample_keep_every_n_))
    {
      out.poses.push_back(in.poses[i]);
    }
    if (out.poses.back().pose.position.x != in.poses.back().pose.position.x ||
        out.poses.back().pose.position.y != in.poses.back().pose.position.y)
    {
      out.poses.push_back(in.poses.back());
    }
    return out;
  }

  if (downsample_min_dist_ <= 0.0) return in;

  nav_msgs::msg::Path out;
  out.header = in.header;
  out.poses.push_back(in.poses.front());

  auto last = in.poses.front();
  for (size_t i = 1; i + 1 < in.poses.size(); ++i) {
    if (distXY(in.poses[i], last) >= downsample_min_dist_) {
      out.poses.push_back(in.poses[i]);
      last = in.poses[i];
    }
  }

  out.poses.push_back(in.poses.back());
  return out;
}

void CoverageServer::runPipeline(const std::shared_ptr<GoalHandleCoverAllMap> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const std::string mode = toLower(goal->order_mode);

  auto feedback = std::make_shared<CoverAllMap::Feedback>();
  auto result = std::make_shared<CoverAllMap::Result>();

  // publish feedback only on change
  uint16_t last_status = CoverAllMap::Feedback::NONE;
  auto publish_state = [&](uint16_t code) {
    if (code == last_status) return;
    last_status = code;
    feedback->status_code = code;
    goal_handle->publish_feedback(feedback);
  };

  auto finish_preempted = [&]() {
    // Preemption is NOT a client cancel request -> must NOT call canceled()
    result->error_code = CoverAllMap::Result::UNKNOWN; 
    result->error_msg = "Preempted by a new goal";
    // ABORT is a valid transition from EXECUTING
    try {
      goal_handle->abort(result);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_WARN(get_logger(), "finish_preempted abort() failed: %s", e.what());
    }
  };

  auto finish_canceled = [&]() {
    result->error_code = CoverAllMap::Result::UNKNOWN;
    result->error_msg = "Canceled";
    // Only call canceled() if the goal is actually canceling
    try {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        // fallback safety: abort if something races
        goal_handle->abort(result);
      }
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_WARN(get_logger(), "finish_canceled finalize failed: %s", e.what());
    }
  };

  auto check_stop = [&]() -> bool {
    if (stop_worker_.load()) return true;
    if (goal_handle->is_canceling()) return true;
    if (preempt_requested_.load()) return true;
    return false;
  };

  // ---- Creating poses ----
  publish_state(CoverAllMap::Feedback::CREATING_POSES);

  nav_msgs::msg::OccupancyGrid::SharedPtr map, costmap;
  if (!waitForMapCostmap(map, costmap, wait_grid_timeout_sec_, goal_handle)) {
    if (preempt_requested_.load()) { finish_preempted(); return; }
    if (goal_handle->is_canceling()) { finish_canceled(); return; }
    result->error_code = CoverAllMap::Result::TIMEOUT;
    result->error_msg = "Timeout waiting for /map and /costmap";
    goal_handle->abort(result);
    return;
  }

  if (check_stop()) {
    if (preempt_requested_.load()) { finish_preempted(); return; }
    finish_canceled();
    return;
  }

  geometry_msgs::msg::PoseArray poses = poses_creator_->create(*map, 
                                                              *costmap, 
                                                              mode,
                                                              goal->enable_serpentine,
                                                              goal->columns_left_to_right,
                                                              goal->rows_bottom_to_top,
                                                              goal->downsample_step_x,
                                                              goal->downsample_step_y,
                                                              goal->map_occ_threshold,
                                                              goal->costmap_occ_threshold);

  poses.header.stamp = toTimeMsg(now());
  if (poses.header.frame_id.empty()) poses.header.frame_id = "map";

  if (poses.poses.empty()) {
    result->error_code = CoverAllMap::Result::FAIL_TO_PLAN;
    result->error_msg = "No valid coverage poses found";
    goal_handle->abort(result);
    return;
  }

  if (graph_nodes_pub_ && graph_nodes_pub_->is_activated()) {
    graph_nodes_pub_->publish(poses);
  }

  // ---- Planning ----
  publish_state(CoverAllMap::Feedback::PLANNING);

  if (!compute_client_->wait_for_action_server(std::chrono::seconds(2))) {
    result->error_code = CoverAllMap::Result::FAIL_TO_PLAN;
    result->error_msg = "ComputePathThroughPoses action server not available";
    goal_handle->abort(result);
    return;
  }

  Compute::Goal compute_goal;
  const auto stamp_msg = toTimeMsg(now());
  const std::string frame_id = poses.header.frame_id;

  if (set_start_from_first_pose_) {
    geometry_msgs::msg::PoseStamped start;
    start.header.stamp = stamp_msg;
    start.header.frame_id = frame_id;
    start.pose = poses.poses.front();
    compute_goal.start = start;
  }

  setPlannerId(compute_goal, planner_id_, 0);

  {
    std::lock_guard<std::mutex> lk(goal_list_mutex_);
    goal_list_.resize(0);
    for (const auto & p : poses.poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp_msg;
      ps.header.frame_id = frame_id;
      ps.pose = p;
      goal_list_.push_back(ps);
    }
    current_index_ = 0;
  }
  setComputeGoals(compute_goal, goal_list_, stamp_msg, frame_id, 0);

  auto compute_goal_future = compute_client_->async_send_goal(compute_goal);

  // wait goal-handle in small steps to allow preempt/cancel
  {
    const auto start_t = now();
    while (rclcpp::ok()) {
      if (check_stop()) {
        if (preempt_requested_.load()) { finish_preempted(); return; }
        finish_canceled(); return;
      }
      if (compute_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        break;
      }
      if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
        result->error_code = CoverAllMap::Result::TIMEOUT;
        result->error_msg = "Timeout sending ComputePathThroughPoses goal";
        goal_handle->abort(result);
        return;
      }
    }
  }

  auto compute_goal_handle = compute_goal_future.get();
  if (!compute_goal_handle) {
    result->error_code = CoverAllMap::Result::FAIL_TO_PLAN;
    result->error_msg = "ComputePathThroughPoses goal rejected";
    goal_handle->abort(result);
    return;
  }

  auto compute_result_future = compute_client_->async_get_result(compute_goal_handle);

  // wait for result with preempt/cancel checks
  {
    const auto start_t = now();
    while (rclcpp::ok()) {
      if (check_stop()) {
        compute_client_->async_cancel_goal(compute_goal_handle);
        if (preempt_requested_.load()) { finish_preempted(); return; }
        finish_canceled(); return;
      }
      if (compute_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
        break;
      }
      if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
        compute_client_->async_cancel_goal(compute_goal_handle);
        result->error_code = CoverAllMap::Result::TIMEOUT;
        result->error_msg = "Timeout waiting ComputePathThroughPoses result";
        goal_handle->abort(result);
        return;
      }
    }
  }

  auto compute_wrapped = compute_result_future.get();
  if (compute_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !compute_wrapped.result || compute_wrapped.result->path.poses.empty())
  {
    result->error_code = CoverAllMap::Result::FAIL_TO_PLAN;
    result->error_msg = "ComputePathThroughPoses failed or returned empty path";
    goal_handle->abort(result);
    return;
  }

  // Create timer to update current index based on robot pose
  if (!timer_) {
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&CoverageServer::updateRobotPose, this), update_pose_cb_group_);
  }

  nav_msgs::msg::Path path = compute_wrapped.result->path;
  path.header.stamp = stamp_msg;
  path.header.frame_id = frame_id;

  nav_msgs::msg::Path ds_path = downsamplePath(path);
  ds_path.header.stamp = toTimeMsg(now());
  ds_path.header.frame_id = frame_id;

  if (debug_path_pub_ && debug_path_pub_->is_activated()) {
    debug_path_pub_->publish(ds_path);
  }

  // ---- Covering ----
  publish_state(CoverAllMap::Feedback::COVERING);

  if (!follow_client_->wait_for_action_server(std::chrono::seconds(2))) {
    result->error_code = CoverAllMap::Result::FAIL_TO_FOLLOW_PATH;
    result->error_msg = "FollowPath action server not available";
    goal_handle->abort(result);
    return;
  }

  Follow::Goal follow_goal;
  follow_goal.path = ds_path;
  setControllerId(follow_goal, controller_id_, 0);
  setGoalCheckerId(follow_goal, goal_checker_id_, 0);

  auto follow_goal_future = follow_client_->async_send_goal(follow_goal);

  // wait goal-handle in small steps
  {
    const auto start_t = now();
    while (rclcpp::ok()) {
      if (check_stop()) {
        if (preempt_requested_.load()) { finish_preempted(); return; }
        finish_canceled(); return;
      }
      if (follow_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        break;
      }
      if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
        result->error_code = CoverAllMap::Result::TIMEOUT;
        result->error_msg = "Timeout sending FollowPath goal";
        goal_handle->abort(result);
        return;
      }
    }
  }

  auto follow_goal_handle = follow_goal_future.get();
  if (!follow_goal_handle) {
    result->error_code = CoverAllMap::Result::FAIL_TO_FOLLOW_PATH;
    result->error_msg = "FollowPath goal rejected";
    goal_handle->abort(result);
    return;
  }

  auto follow_result_future = follow_client_->async_get_result(follow_goal_handle);

  // wait result with preempt/cancel checks
  {
    const auto start_t = now();
    while (rclcpp::ok()) {
      if (check_stop()) {
        follow_client_->async_cancel_goal(follow_goal_handle);
        if (preempt_requested_.load()) { finish_preempted(); return; }
        finish_canceled(); return;
      }
      if (follow_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
        break;
      }
      if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
        follow_client_->async_cancel_goal(follow_goal_handle);
        result->error_code = CoverAllMap::Result::TIMEOUT;
        result->error_msg = "Timeout waiting FollowPath result";
        goal_handle->abort(result);
        return;
      }
    }
  }

  auto follow_wrapped = follow_result_future.get();
  if (follow_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    bool success_recovered = false;

    while(retries_on_failure_ != 0 || retries_on_failure_ == -1) {
      bool continue_flag = false;
      if (retries_on_failure_ > 0) { 
        retries_on_failure_--; 
      }

      publish_state(CoverAllMap::Feedback::RECOVERING);

      geometry_msgs::msg::PoseStamped current_robot_pose;
      if (costmap_ros_->getRobotPose(current_robot_pose)) {
        // Create new path from current position to remaining goals
        std::vector<geometry_msgs::msg::PoseStamped> remaining_goal_list;
        remaining_goal_list.reserve(goal_list_.size() - current_index_ + 1);
        remaining_goal_list.push_back(current_robot_pose);
        
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
        unsigned int mx = 0;
        unsigned int my = 0;
        bool use_radius = costmap_ros_->getUseRadius();
        
        for (size_t i = current_index_ + 1; i < goal_list_.size(); ++i) {
          unsigned int cost = nav2_costmap_2d::FREE_SPACE;
          if (use_radius) {
            if (costmap_->worldToMap(goal_list_[i].pose.position.x, goal_list_[i].pose.position.y, mx, my)) {
              cost = costmap_->getCost(mx, my);
            } else {
              cost = nav2_costmap_2d::LETHAL_OBSTACLE;
            }
          } else {
            nav2_costmap_2d::Footprint footprint = costmap_ros_->getRobotFootprint();
            auto theta = tf2::getYaw(goal_list_[i].pose.orientation);
            cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
              goal_list_[i].pose.position.x, goal_list_[i].pose.position.y, theta, footprint));
          }
          if (cost >= static_cast<unsigned int>(recovery_obstacle_max_cost_)) {
            continue;
          } else {
            remaining_goal_list.push_back(goal_list_[i]);
          }
        }

        {
          std::lock_guard<std::mutex> lk(goal_list_mutex_);
          goal_list_.resize(remaining_goal_list.size());
          goal_list_ = remaining_goal_list;
          current_index_ = 0;
        }

        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = toTimeMsg(now());
        msg.header.frame_id = frame_id;
        for (const auto & p: goal_list_) {
          msg.poses.push_back(p.pose);
        }

        if (graph_nodes_pub_ && graph_nodes_pub_->is_activated()) {
          graph_nodes_pub_->publish(msg);
        }

        publish_state(CoverAllMap::Feedback::PLANNING);

        // Re-plan path
        if (!compute_client_->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_WARN(get_logger(), "ComputePathThroughPoses action server not available during recovery");
          continue;
        }

        Compute::Goal recovery_compute_goal;
        const auto recovery_stamp_msg = toTimeMsg(now());

        setPlannerId(recovery_compute_goal, planner_id_, 0);

        setComputeGoals(recovery_compute_goal, remaining_goal_list, recovery_stamp_msg, frame_id, 0);

        auto recovery_compute_goal_future = compute_client_->async_send_goal(recovery_compute_goal);

        // wait goal-handle in small steps to allow preempt/cancel
        continue_flag = false;
        {
          const auto start_t = now();
          while (rclcpp::ok()) {
            if (check_stop()) {
              if (preempt_requested_.load()) { finish_preempted(); return; }
              finish_canceled(); return;
            }
            if (recovery_compute_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
              break;
            }
            if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
              RCLCPP_WARN(get_logger(), "ComputePathThroughPoses action server not available during recovery");
              continue_flag = true;
              break;
            }
          }
        }if (continue_flag) {continue;}

        auto recovery_compute_goal_handle = recovery_compute_goal_future.get();
        if (!recovery_compute_goal_handle) {
          RCLCPP_WARN(get_logger(), "ComputePathThroughPoses goal rejected during recovery");
          continue;
        }

        auto recovery_compute_result_future = compute_client_->async_get_result(recovery_compute_goal_handle);

        // wait for result with preempt/cancel checks
        continue_flag = false;
        {
          const auto start_t = now();
          while (rclcpp::ok()) {
            if (check_stop()) {
              compute_client_->async_cancel_goal(recovery_compute_goal_handle);
              if (preempt_requested_.load()) { finish_preempted(); return; }
              finish_canceled(); return;
            }
            if (recovery_compute_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
              break;
            }
            if (compute_timeout_sec_ > 0.0 && (now() - start_t).seconds() > compute_timeout_sec_) {
              compute_client_->async_cancel_goal(recovery_compute_goal_handle);
              RCLCPP_WARN(get_logger(), "Timeout waiting ComputePathThroughPoses result during recovery");
              continue_flag = true;
              break;
            }
          }
        }if (continue_flag) {continue;}

        auto recovery_compute_wrapped = recovery_compute_result_future.get();
        if (recovery_compute_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !recovery_compute_wrapped.result || recovery_compute_wrapped.result->path.poses.empty())
        {
          RCLCPP_WARN(get_logger(), "ComputePathThroughPoses failed or returned empty path during recovery");
          continue;
        }

        nav_msgs::msg::Path recovery_path = recovery_compute_wrapped.result->path;
        recovery_path.header.stamp = stamp_msg;
        recovery_path.header.frame_id = frame_id;

        ds_path = downsamplePath(recovery_path);
        ds_path.header.stamp = toTimeMsg(now());
        ds_path.header.frame_id = frame_id;

        if (debug_path_pub_ && debug_path_pub_->is_activated()) {
          debug_path_pub_->publish(ds_path);
        }

        // ---- Covering ----
        publish_state(CoverAllMap::Feedback::COVERING);

        if (!follow_client_->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_WARN(get_logger(), "FollowPath action server not available during recovery");
          continue;
        }

        Follow::Goal recovery_follow_goal;
        recovery_follow_goal.path = ds_path;
        setControllerId(recovery_follow_goal, controller_id_, 0);
        setGoalCheckerId(recovery_follow_goal, goal_checker_id_, 0);

        auto recovery_follow_goal_future = follow_client_->async_send_goal(recovery_follow_goal);

        // wait goal-handle in small steps
        continue_flag = false;
        {
          const auto start_t = now();
          while (rclcpp::ok()) {
            if (check_stop()) {
              if (preempt_requested_.load()) { finish_preempted(); return; }
              finish_canceled(); return;
            }
            if (recovery_follow_goal_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
              break;
            }
            if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
              RCLCPP_WARN(get_logger(), "FollowPath action server not available during recovery");
              continue_flag = true;
              break;
            }
          }
        }if (continue_flag) {continue;}

        auto recovery_follow_goal_handle = recovery_follow_goal_future.get();
        if (!recovery_follow_goal_handle) {
          RCLCPP_WARN(get_logger(), "FollowPath goal rejected during recovery");
          continue;
        }

        auto recovery_follow_result_future = follow_client_->async_get_result(recovery_follow_goal_handle);

        // wait result with preempt/cancel checks
        continue_flag = false;
        {
          const auto start_t = now();
          while (rclcpp::ok()) {
            if (check_stop()) {
              follow_client_->async_cancel_goal(recovery_follow_goal_handle);
              if (preempt_requested_.load()) { finish_preempted(); return; }
              finish_canceled(); return;
            }
            if (recovery_follow_result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
              break;
            }
            if (follow_timeout_sec_ > 0.0 && (now() - start_t).seconds() > follow_timeout_sec_) {
              follow_client_->async_cancel_goal(recovery_follow_goal_handle);
              RCLCPP_WARN(get_logger(), "Timeout waiting FollowPath result during recovery");
              continue_flag = true;
              break;
            }
          }
        } if (continue_flag) {continue;}

        auto recovery_follow_wrapped = recovery_follow_result_future.get();
        if (recovery_follow_wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "FollowPath failed during recovery");
          continue;
        } 
        else {
          success_recovered = true;
          break;
        }
      }
    }
    if (!success_recovered) {
      result->error_code = CoverAllMap::Result::FAIL_TO_FOLLOW_PATH;
      result->error_msg = "FollowPath failed";
      goal_handle->abort(result);
      if (timer_) {
        timer_->cancel();
        timer_.reset();
      }
      return;
    }
  }

  // Success
  result->error_code = CoverAllMap::Result::SUCCESS;
  result->error_msg = "";
  goal_handle->succeed(result);
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

}  // namespace nav2_coverage

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_coverage::CoverageServer)

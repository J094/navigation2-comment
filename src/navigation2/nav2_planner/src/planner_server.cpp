// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // 声明节点的参数
  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  // 获取 plugin 的名称
  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      // 这里声明一下默认 plugins 参数
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // 配置 global costmap, 这里是建一个新的
  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");

  // 开启 costmap 线程
  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

PlannerServer::~PlannerServer()
{
  // 清空 planner 和 costmap 线程
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // 配置 costmap_ros
  // TODO: 貌似 Costmap2DROS 没有 configure 函数?
  costmap_ros_->configure();
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  // 获取 tf buffer
  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  // 本节点的 shared_ptr
  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      // 获取 plugin 的类型
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      // 创建 planner 实例
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      // 然后就是配置 planner 和 插入到 planners_ 里面去
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // 这玩意是所有规划器的名称连接起来
  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  // 获取规划器的频率
  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // 初始化发布
  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // 创建路径规划的 action servers
  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    shared_from_this(),
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // 记过各个东西
  plan_publisher_->on_activate();
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->activate();

  // map 的迭代器, 激活每一个规划器
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  // 拿到本身的 shared_ptr
  auto node = shared_from_this();

  // 检查 path 是否 valid 的服务
  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // 给动态参数修改增加回调
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // 创建关联
  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();
  costmap_ros_->cleanup();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // 这里循环等待
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  // 如果发生抢占, 就使用 pending goal
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  // 如果 goal 里使用开始点, 就设置开始点
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    // 如果获取不到机器人位姿就中断
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  // 路径不能为空
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %zu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlanThroughPoses()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    std::vector<geometry_msgs::msg::PoseStamped>::iterator goal_iter;
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        curr_start = goal->goals[i - 1];
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      // check path for validity
      if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id)) {
        return;
      }

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %zu points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
}

void
PlannerServer::computePlan()
{
  // 在计算路径的时候不允许修改动态参数
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  // 用于计算路径规划花费的时间
  auto start_time = steady_clock_.now();

  // 拿到 goal 并创建 result 准备接收结果
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    // action server 应该是激活的, 并且没有被取消
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    // 等待 costmap 获取到
    waitForCostmap();

    // 检查是否有抢占
    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // 如果提供了开始点就使用开始点, 否则使用当前机器人位姿作为 start
    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_pose_, goal, start)) {
      return;
    }

    // 把 start 和 goal 都转换到世界坐标系
    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose)) {
      return;
    }

    // 计算路径
    // planner id 是有 goal 指定的
    result->path = getPlan(start, goal_pose, goal->planner_id);

    // 验证路径
    if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // 发布路径
    // Publish the plan for visualization purposes
    publishPlan(result->path);

    // 保存规划所用的时间
    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    // 如果规划时间大于规划周期时间, 需要发出警告
    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    // 宣布成功完成规划
    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // 先判断有指定的规划器
  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      // 如果没有指定的规划器就默认使用第一个规划器
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  // 如果规划失败, 返回空路径
  return nav_msgs::msg::Path();
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  // 创建 unique_ptr, 然后用 move 发布, 避免二次拷贝
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  // 需要发布者激活中, 并且有订阅者订阅才发布
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  // 开始默认 true
  response->is_valid = true;

  // 首先 path 不能为空
  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  // 计算最近点的 index
  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    // 检查地图上点的 cost, 如果是不可通行的, 就是 invalid
    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    unsigned int mx = 0;
    unsigned int my = 0;
    // 从最近点开始, 避免重复计算
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      // 获取 cost
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        // 只要有一个点是遮挡的, 这条路径不是 valid
        response->is_valid = false;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // 修改动态参数的时候需要锁住
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // 这里貌似只配置 expected planner frequency
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)

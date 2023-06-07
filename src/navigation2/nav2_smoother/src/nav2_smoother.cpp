// Copyright (c) 2019 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_smoother/nav2_smoother.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

namespace nav2_smoother
{

SmootherServer::SmootherServer(const rclcpp::NodeOptions & options)
: LifecycleNode("smoother_server", "", options),
  lp_loader_("nav2_core", "nav2_core::Smoother"),
  default_ids_{"simple_smoother"},
  default_types_{"nav2_smoother::SimpleSmoother"}
{
  RCLCPP_INFO(get_logger(), "Creating smoother server");

  // costmap 的 topic
  declare_parameter(
    "costmap_topic", rclcpp::ParameterValue(
      std::string(
        "global_costmap/costmap_raw")));
  // footprint 表示机器人的几何, 是否碰撞
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(
      std::string("global_costmap/published_footprint")));
  // 机器人的 frame 名称, 默认 base_link
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  // tf 容忍阈值
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  // 用到的 smoother plugin 名称
  declare_parameter("smoother_plugins", default_ids_);
}

SmootherServer::~SmootherServer()
{
  smoothers_.clear();
}

nav2_util::CallbackReturn
SmootherServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring smoother server");

  // 二话不说, 先把自己搞成 shared_ptr 再用
  auto node = shared_from_this();

  // 获取 smoother id
  get_parameter("smoother_plugins", smoother_ids_);
  if (smoother_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  // tf buffer 用于获取位姿
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // 各种 topic 和订阅
  std::string costmap_topic, footprint_topic, robot_base_frame;
  double transform_tolerance;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("footprint_topic", footprint_topic);
  this->get_parameter("transform_tolerance", transform_tolerance);
  this->get_parameter("robot_base_frame", robot_base_frame);
  costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame, transform_tolerance);

  collision_checker_ =
    std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, this->get_name());

  // 加载 plugins
  if (!loadSmootherPlugins()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // smooth path 发布初始化
  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan_smoothed", 1);

  // 创建 ros action server 等待请求
  // Create the action server that we implement with our smoothPath method
  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "smooth_path",
    std::bind(&SmootherServer::smoothPlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

bool SmootherServer::loadSmootherPlugins()
{
  auto node = shared_from_this();

  smoother_types_.resize(smoother_ids_.size());

  // 保存 smoother_types_
  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    try {
      smoother_types_[i] =
        nav2_util::get_plugin_type_param(node, smoother_ids_[i]);
      // 创建 smoother 实例
      nav2_core::Smoother::Ptr smoother =
        lp_loader_.createUniqueInstance(smoother_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created smoother : %s of type %s",
        smoother_ids_[i].c_str(), smoother_types_[i].c_str());
      // configure 每一个 smoother plugin
      smoother->configure(
        node, smoother_ids_[i], tf_, costmap_sub_,
        footprint_sub_);
      // 保存到 map 中
      smoothers_.insert({smoother_ids_[i], smoother});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create smoother. Exception: %s",
        ex.what());
      return false;
    }
  }

  for (size_t i = 0; i != smoother_ids_.size(); i++) {
    smoother_ids_concat_ += smoother_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "Smoother Server has %s smoothers available.",
    smoother_ids_concat_.c_str());

  return true;
}

nav2_util::CallbackReturn
SmootherServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // 激活发布
  plan_publisher_->on_activate();
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    // 激活每一个 smoother
    it->second->activate();
  }
  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // 反激活
  action_server_->deactivate();
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->deactivate();
  }
  plan_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // 重置所有
  // Cleanup the helper classes
  SmootherMap::iterator it;
  for (it = smoothers_.begin(); it != smoothers_.end(); ++it) {
    it->second->cleanup();
  }
  smoothers_.clear();

  // Release any allocated resources
  action_server_.reset();
  plan_publisher_.reset();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  collision_checker_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SmootherServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool SmootherServer::findSmootherId(
  const std::string & c_name,
  std::string & current_smoother)
{
  // 检查是否存在, 如果不存在就设置为第一个默认的 smoother
  // 如果存在就是 c_name
  if (smoothers_.find(c_name) == smoothers_.end()) {
    if (smoothers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No smoother was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.",
        smoother_ids_concat_.c_str());
      current_smoother = smoothers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "SmoothPath called with smoother name %s, "
        "which does not exist. Available smoothers are: %s.",
        c_name.c_str(), smoother_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected smoother: %s.", c_name.c_str());
    current_smoother = c_name;
  }

  return true;
}

void SmootherServer::smoothPlan()
{
  // 记录开始 smoothing 的时间
  auto start_time = steady_clock_.now();

  RCLCPP_INFO(get_logger(), "Received a path to smooth.");

  auto result = std::make_shared<Action::Result>();
  try {
    // 获取 smoother_id 选择指定的 smoother
    std::string c_name = action_server_->get_current_goal()->smoother_id;
    std::string current_smoother;
    if (findSmootherId(c_name, current_smoother)) {
      current_smoother_ = current_smoother;
    } else {
      action_server_->terminate_current();
      return;
    }

    // 执行 smoothing
    // Perform smoothing
    // 先拿 goal
    auto goal = action_server_->get_current_goal();
    result->path = goal->path;
    // 调用 smoother 来执行 smoothing
    result->was_completed = smoothers_[current_smoother_]->smooth(
      result->path, goal->max_smoothing_duration);
    // 计算时间花费
    result->smoothing_duration = steady_clock_.now() - start_time;

    if (!result->was_completed) {
      RCLCPP_INFO(
        get_logger(),
        "Smoother %s did not complete smoothing in specified time limit"
        "(%lf seconds) and was interrupted after %lf seconds",
        current_smoother_.c_str(),
        rclcpp::Duration(goal->max_smoothing_duration).seconds(),
        rclcpp::Duration(result->smoothing_duration).seconds());
    }
    // 如果完成了就发布
    plan_publisher_->publish(result->path);

    // 检查结果 path 是否有碰撞
    // Check for collisions
    if (goal->check_for_collisions) {
      geometry_msgs::msg::Pose2D pose2d;
      bool fetch_data = true;
      for (const auto & pose : result->path.poses) {
        pose2d.x = pose.pose.position.x;
        pose2d.y = pose.pose.position.y;
        pose2d.theta = tf2::getYaw(pose.pose.orientation);

        // 如果有一个位姿是碰撞的, smoothing 失败
        if (!collision_checker_->isCollisionFree(pose2d, fetch_data)) {
          RCLCPP_ERROR(
            get_logger(),
            "Smoothed path leads to a collision at x: %lf, y: %lf, theta: %lf",
            pose2d.x, pose2d.y, pose2d.theta);
          action_server_->terminate_current(result);
          return;
        }
        fetch_data = false;
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "Smoother succeeded (time: %lf), setting result",
      rclcpp::Duration(result->smoothing_duration).seconds());

    // 如果检查通过, 返回成功
    // 如果出现异常, 中断所有
    action_server_->succeeded_current(result);
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    action_server_->terminate_current();
    return;
  } catch (std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    action_server_->terminate_current(result);
    return;
  }
}

}  // namespace nav2_smoother

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_smoother::SmootherServer)

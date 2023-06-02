// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace nav2_bt_navigator
{

// 保存一些反馈参数
/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

// 互斥锁, BT navigator 中同时只允许一个 plugin 执行
/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating()
  {
    // NOTE: scoped_lock 会在超出作用域自动解锁
    std::scoped_lock l(mutex_);
    // 不为空表示还在导航中
    return !current_navigator_.empty();
  }

  // 中介函数
  /**
   * @brief Start navigating with a given navigator
   * @param string Name of the navigator to start
   */
  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    // 如果还在其他导航状态, 不能开启下一次导航
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    }
    // 说明开启了新的导航
    current_navigator_ = navigator_name;
  }

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    // 必须还在导航状态中, 才能停止
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    } else {
      // 停止就是将当前导航状态赋空
      current_navigator_ = std::string("");
    }
  }

protected:
  // 当前的 plugin 名称
  std::string current_navigator_;
  // 互斥锁
  std::mutex mutex_;
};

/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template<class ActionT>
class Navigator
{
public:
  // 设置 Ptr 为 shared_ptr
  using Ptr = std::shared_ptr<nav2_bt_navigator::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator()
  {
    // 构造函数初始化 plugin_muxer_ 为空指针
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() = default;

  /**
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param feedback_utils Some utilities useful for navigators to have
   * @param plugin_muxer The muxing object to ensure only one navigator
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   */
  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    nav2_bt_navigator::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
  {
    // parent_node 是一个 weak_ptr, 功能和 shared_ptr 一样, 但是不会增加引用计数
    // 通过 lock() 可以返回指向对象的一个 shared_ptr, 如果对象还存在
    // 如果对象不存在, 则会返回空的 shared_ptr
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // 获取这个 navigator 的默认行为树
    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // 创建行为树 action server
    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Navigator::onLoop, this),
      std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
      std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    // 配置 action 服务
    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    // 拿到黑板, 放些东西进去
    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);  // NOLINT

    // 配置本身
    return configure(parent_node, odom_smoother) && ok;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_activate()
  {
    // 激活 action 服务
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    // 激活本身
    return activate() && ok;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate()
  {
    // 反激活 action 服务
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    // 反激活本身
    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup()
  {
    // 清除 action 服务
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    // 清除本身
    return cleanup() && ok;
  }

  // 虚函数, 需要重写
  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  // 中介函数, 带互斥锁避免异常
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    // 只有没在导航才接收 goal
    bool goal_accepted = goalReceived(goal);

    // 获取到 goal 就开始导航
    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  // 中介函数
  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    // 
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  // 虚函数, 需要重写
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  // 虚函数, 需要重写
  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  // 虚函数, 需要重写
  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  // 虚函数, 需要重写
  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/,
    std::shared_ptr<nav2_util::OdomSmoother>/*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  /**
   * @brief Method to activate any threads involved in execution.
   */
  virtual bool activate() {return true;}

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() {return true;}

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATOR_HPP_

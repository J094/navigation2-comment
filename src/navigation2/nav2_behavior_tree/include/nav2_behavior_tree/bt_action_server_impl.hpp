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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
BtActionServer<ActionT>::BtActionServer(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & action_name,
  const std::vector<std::string> & plugin_lib_names,
  const std::string & default_bt_xml_filename,
  OnGoalReceivedCallback on_goal_received_callback,
  OnLoopCallback on_loop_callback,
  OnPreemptCallback on_preempt_callback,
  OnCompletionCallback on_completion_callback)
: action_name_(action_name),
  default_bt_xml_filename_(default_bt_xml_filename),
  plugin_lib_names_(plugin_lib_names),
  node_(parent),
  on_goal_received_callback_(on_goal_received_callback),
  on_loop_callback_(on_loop_callback),
  on_preempt_callback_(on_preempt_callback),
  on_completion_callback_(on_completion_callback)
{
  // lock 临时通过 weak_ptr 生成 shared_ptr
  // node 会被自动释放
  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 拿参数
  // Declare this node's parameters
  if (!node->has_parameter("bt_loop_duration")) {
    node->declare_parameter("bt_loop_duration", 10);
  }
  if (!node->has_parameter("default_server_timeout")) {
    node->declare_parameter("default_server_timeout", 20);
  }
}

template<class ActionT>
BtActionServer<ActionT>::~BtActionServer()
{}

template<class ActionT>
bool BtActionServer<ActionT>::on_configure()
{
  // 生成临时 shared_ptr
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // client 节点名称定义
  // Name client node after action name
  std::string client_node_name = action_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  // NOTE: "--ros-args" 表示 ros 参数, "-r" 表示重映射, "__node:=" 表示重新定义节点名称, "--" 表示参数列表结束
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") +
      std::string(node->get_name()) + "_" + client_node_name + "_rclcpp_node",
      "--"});

  // client 节点原来在此, 最后会被放到黑板的 node 中, 后续给各个 action server 发请求的基础
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  // action 服务
  action_server_ = std::make_shared<ActionServer>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    action_name_, std::bind(&BtActionServer<ActionT>::executeCallback, this));

  // BT 和服务的时间阈值
  // Get parameters for BT timeouts
  int timeout;
  node->get_parameter("bt_loop_duration", timeout);
  bt_loop_duration_ = std::chrono::milliseconds(timeout);
  node->get_parameter("default_server_timeout", timeout);
  default_server_timeout_ = std::chrono::milliseconds(timeout);

  // 创建 bt_ 为 BT 引擎, 用于实际执行树
  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // 创建黑板
  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);  // NOLINT

  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_activate()
{
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }
  action_server_->activate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_deactivate()
{
  action_server_->deactivate();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::on_cleanup()
{
  client_node_.reset();
  action_server_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();
  return true;
}

template<class ActionT>
bool BtActionServer<ActionT>::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // 如果文件为空, 拿默认的文件
  // Empty filename is default for backward compatibility
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // 如果行为树没有变化, 不需要重新加载
  // Use previous BT if it is the existing one
  if (current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  // 把 xml 文件转成 string
  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  try {
    // 通过 string 穿换成 BT
    tree_ = bt_->createTreeFromText(xml_string, blackboard_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception when loading BT: %s", e.what());
    return false;
  }

  // 使用 node 和 tree 创建 logger, 发布相关记录
  topic_logger_ = std::make_unique<RosTopicLogger>(client_node_, tree_);

  // 更新当前树 xml 文件
  current_bt_xml_filename_ = filename;
  return true;
}

// 用于在 ros action server 里实际执行任务的回调
template<class ActionT>
void BtActionServer<ActionT>::executeCallback()
{
  // 先获取 goal, 失败则 terminate
  // 实际是从 ros action goal handle 中获取 goal
  // 这里指定的是 current handle
  if (!on_goal_received_callback_(action_server_->get_current_goal())) {
    action_server_->terminate_current();
    return;
  }

  // 定义 cancelRequested
  auto is_canceling = [&]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
        return true;
      }
      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
        return true;
      }
      return action_server_->is_cancel_requested();
    };

  // 定义 onLoop
  // 每个循环都检查 is_preempt_requested
  // 如果 pending 有东西, 就是 preempt requested 的状态
  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested() && on_preempt_callback_) {
        // 这里指定的是 pending handle, 通过它获取 goal
        on_preempt_callback_(action_server_->get_pending_goal());
      }
      topic_logger_->flush();
      // 正常的每个 loop 都干的事
      on_loop_callback_();
    };

  // 这里执行行为树
  // 这里传入了 on_loop, 意味着是每 tick 一次就干一次
  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);

  //  halt 所有节点
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());

  // 检查结果
  // Give server an opportunity to populate the result message or simple give
  // an indication that the action is complete.
  auto result = std::make_shared<typename ActionT::Result>();
  on_completion_callback_(result, rc);

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(logger_, "Goal succeeded");
      action_server_->succeeded_current(result);
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(logger_, "Goal failed");
      action_server_->terminate_current(result);
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(logger_, "Goal canceled");
      action_server_->terminate_all(result);
      break;
  }
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_SERVER_IMPL_HPP_

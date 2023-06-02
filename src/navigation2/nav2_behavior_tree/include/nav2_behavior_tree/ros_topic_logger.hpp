// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_
#define NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include <memory>
#include <utility>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"
#include "tf2_ros/buffer_interface.h"

namespace nav2_behavior_tree
{

// 在 BT 状态变化发布 BT logs, 记录了每一次的状态变化
/**
 * @brief A class to publish BT logs on BT status change
 */
class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RosTopicLogger
   * @param ros_node Weak pointer to parent rclcpp::Node
   * @param tree BT to monitor
   */
  RosTopicLogger(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
  : StatusChangeLogger(tree.rootNode())
  {
    // 通过 weak_ptr 拿 shared_ptr
    auto node = ros_node.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    // 创建 log 的 publisher
    log_pub_ = node->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
      "behavior_tree_log",
      rclcpp::QoS(10));
  }

  /**
   * @brief Callback function which is called each time BT changes status
   * @param timestamp Timestamp of BT status change
   * @param node Node that changed status
   * @param prev_status Previous status of the node
   * @param status Current status of the node
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    nav2_msgs::msg::BehaviorTreeStatusChange event;

    // BT timestamps are a duration since the epoch. Need to convert to a time_point
    // before converting to a msg.
    // NOTE: 把时间转换到 msg
    event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
    event.node_name = node.name();
    event.previous_status = toStr(prev_status, false);
    event.current_status = toStr(status, false);
    // 用 move 避免复制
    event_log_.push_back(std::move(event));

    RCLCPP_DEBUG(
      logger_, "[%.3f]: %25s %s -> %s",
      std::chrono::duration<double>(timestamp).count(),
      node.name().c_str(),
      toStr(prev_status, true).c_str(),
      toStr(status, true).c_str() );
  }

  /**
   * @brief Clear log buffer if any
   */
  void flush() override
  {
    // 如果不为空, 发布 log
    // NOTE: 只是在这里发布吗? 还要清空 event_log_?
    if (!event_log_.empty()) {
      auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
      log_msg->timestamp = clock_->now();
      log_msg->event_log = event_log_;
      log_pub_->publish(std::move(log_msg));
      event_log_.clear();
    }
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace nav2_behavior_tree

#endif   // NAV2_BEHAVIOR_TREE__ROS_TOPIC_LOGGER_HPP_

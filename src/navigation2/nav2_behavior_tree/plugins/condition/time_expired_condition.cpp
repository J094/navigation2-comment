// Copyright (c) 2019 Intel Corporation
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

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

namespace nav2_behavior_tree
{

TimeExpiredCondition::TimeExpiredCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0)
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
}

BT::NodeStatus TimeExpiredCondition::tick()
{
  // 如果处于 IDLE, 意味没有 expired, 返回 FAILURE
  if (status() == BT::NodeStatus::IDLE) {
    start_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }

  // 计算这一个循环所经历的时间
  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  // 如果经历的时间少于 period_ 则返回 FAILURE
  if (seconds < period_) {
    return BT::NodeStatus::FAILURE;
  }

  // 如果时间超过了, 重置开始时间, 返回 SUCCESS
  // 该节点的作用就是保证一定时间的运行才返回 SUCCESS
  start_ = node_->now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}

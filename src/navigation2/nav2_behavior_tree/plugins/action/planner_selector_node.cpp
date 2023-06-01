// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/planner_selector_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PlannerSelector::PlannerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  // 拿到节点, 管理回调
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  getInput("topic_name", topic_name_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  // 创建订阅
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  planner_selector_sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name_,
    qos,
    std::bind(&PlannerSelector::callbackPlannerSelect, this, _1),
    sub_option);
}

BT::NodeStatus PlannerSelector::tick()
{
  // 等等订阅回调
  callback_group_executor_.spin_some();

  // 如果为空, 用默认规划器
  // This behavior always use the last selected planner received from the topic input.
  // When no input is specified it uses the default planner.
  // If the default planner is not specified then we work in "required planner mode":
  // In this mode, the behavior returns failure if the planner selection is not received from
  // the topic input.
  if (last_selected_planner_.empty()) {
    std::string default_planner;
    getInput("default_planner", default_planner);
    if (default_planner.empty()) {
      return BT::NodeStatus::FAILURE;
    } else {
      last_selected_planner_ = default_planner;
    }
  }

  // 输出 planner 到 selected_planner
  setOutput("selected_planner", last_selected_planner_);

  return BT::NodeStatus::SUCCESS;
}

void
PlannerSelector::callbackPlannerSelect(const std_msgs::msg::String::SharedPtr msg)
{
  // 拿到数据保存
  last_selected_planner_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PlannerSelector>("PlannerSelector");
}

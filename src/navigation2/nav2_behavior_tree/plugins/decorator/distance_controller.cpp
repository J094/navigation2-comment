// Copyright (c) 2018 Intel Corporation
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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/distance_controller.hpp"

namespace nav2_behavior_tree
{

DistanceController::DistanceController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(1.0),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  first_time_(false)
{
  getInput("distance", distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus DistanceController::tick()
{
  // IDLE 状态下获取初始位姿
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    if (!nav2_util::getCurrentPose(
        start_pose_, *tf_, global_frame_, robot_base_frame_,
        transform_tolerance_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
      return BT::NodeStatus::FAILURE;
    }
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // 获取当前位姿
  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // 计算运行距离
  // Get euclidean distance
  auto travelled = nav2_util::geometry_utils::euclidean_distance(
    start_pose_.pose, current_pose.pose);

  // 运行距离大于 distance 就 tick 一次
  // 子节点如果在运行中则不停 tick
  // The child gets ticked the first time through and every time the threshold
  // distance is crossed. In addition, once the child begins to run, it is
  // ticked each time 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    travelled >= distance_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        // 如果成功了, 则更新开始位姿
        if (!nav2_util::getCurrentPose(
            start_pose_, *tf_, global_frame_, robot_base_frame_,
            transform_tolerance_))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
          return BT::NodeStatus::FAILURE;
        }
        // 如果成功更新了开始位姿，则返回 SUCCESS
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DistanceController>("DistanceController");
}

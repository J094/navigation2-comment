// Copyright (c) 2021 Samsung Research
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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"

namespace nav2_bt_navigator
{

bool
NavigateToPoseNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  // 黑板 id
  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }

  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  // 创建自己的 client, 节点是父节点, 名字是自己
  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  // 订阅 goal
  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  return true;
}

std::string
NavigateToPoseNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  // 获取默认的 xml 文件
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    // 如果父节点没有, 声明一个参数
    node->declare_parameter<std::string>(
      "default_nav_to_pose_bt_xml",
      pkg_share_dir +
      "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
  }

  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
NavigateToPoseNavigator::cleanup()
{
  // 重置订阅和客户
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool
NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  // 拿到行为树
  auto bt_xml_filename = goal->behavior_tree;

  // 读取行为树
  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  // 初始化 goal
  initializeGoalPose(goal);

  return true;
}

void
NavigateToPoseNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
  // 不做任何操作
}

// BT 引擎 tick 一次 BT 就会 onLoop 一次
void
NavigateToPoseNavigator::onLoop()
{
  // 反馈消息
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  // 拿到当前位姿
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,
    feedback_utils_.global_frame, feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);

  // 拿黑板
  auto blackboard = bt_action_server_->getBlackboard();

  try {
    // 通过 id 在黑板中拿到 path
    // Get current path points
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);

    // 找到距离当前位姿最近的 pose
    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    // 计算剩余距离
    // Calculate distance on the path
    double distance_remaining =
      nav2_util::geometry_utils::calculate_path_length(current_path, find_closest_pose_idx());

    // Default value for time remaining
    rclcpp::Duration estimated_time_remaining = rclcpp::Duration::from_seconds(0.0);

    // 得到当前速度
    // Get current speed
    geometry_msgs::msg::Twist current_odom = odom_smoother_->getTwist();
    double current_linear_speed = std::hypot(current_odom.linear.x, current_odom.linear.y);

    // 通过当前速度估计还需要多长时间能完成
    // Calculate estimated time taken to goal if speed is higher than 1cm/s
    // and at least 10cm to go
    if ((std::abs(current_linear_speed) > 0.01) && (distance_remaining > 0.1)) {
      estimated_time_remaining =
        rclcpp::Duration::from_seconds(distance_remaining / std::abs(current_linear_speed));
    }

    // 反馈消息赋值, 包括剩余距离和剩余时间
    feedback_msg->distance_remaining = distance_remaining;
    feedback_msg->estimated_time_remaining = estimated_time_remaining;
  } catch (...) {
    // Ignore
  }

  // 获取恢复次数
  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);
  // 反馈消息赋值, 包括恢复次数, 当前位姿和已运行时间
  feedback_msg->number_of_recoveries = recovery_count;
  feedback_msg->current_pose = current_pose;
  feedback_msg->navigation_time = clock_->now() - start_time_;

  // 在 onLoop 的最后发布反馈
  bt_action_server_->publishFeedback(feedback_msg);
}

// 这里是拿到了 pending 里的 goal
void
NavigateToPoseNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  // 中断
  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // 相同的 BT, 接受 pending goal
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    // 不相同的 BT, 意味着是一个不同的行为来抢占的, 这里建议 cancel 当前目标, 发送新的目标
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    logger_, "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // 这里拿到新的 goal 了, 重置 recovery 计数
  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // 把新的 goal 放入黑板中, bt action 可以从黑板中获取 goal
  // Update the goal pose on the blackboard
  blackboard->set<geometry_msgs::msg::PoseStamped>(goal_blackboard_id_, goal->pose);
}

void
NavigateToPoseNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // 拿到 goal 直接发送请求
  ActionT::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator

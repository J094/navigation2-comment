// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
#define NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_behaviors
{
using SpinAction = nav2_msgs::action::Spin;

/**
 * @class nav2_behaviors::Spin
 * @brief An action server behavior for spinning in
 */
class Spin : public TimedBehavior<SpinAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::Spin
   */
  Spin();
  ~Spin();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;

  // 最大最小旋转速度
  double min_rotational_vel_;
  double max_rotational_vel_;
  // 旋转加速度限制
  double rotational_acc_lim_;
  // 要求旋转的角度
  double cmd_yaw_;
  // 当前角度
  double prev_yaw_;
  // 相对角度, 已经旋转的角度
  double relative_yaw_;
  // 模拟预测时间
  double simulate_ahead_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_

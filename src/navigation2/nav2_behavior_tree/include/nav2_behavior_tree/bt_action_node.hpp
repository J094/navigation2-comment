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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 */
template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    // 从黑板拿 ros 节点
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    // 创建 callback 集合, 这里设置节点中同时只能有一个 callback 在执行
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    // 给 callback 执行器添加 callback 集合, 同时连接 ros 节点
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // 从黑板中拿需要的参数
    // Get the required items from the blackboard
    bt_loop_duration_ =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // 初始化输入输出消息
    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    // 查看 action 名字是否被 remapped
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    // 创建客户
    createActionClient(action_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   * @param action_name Action name to create client for
   */
  void createActionClient(const std::string & action_name)
  {
    // 这里给 BT action 创建 ros action client
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // 这里是确保服务存在
    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    if (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for 1 s",
        action_name.c_str());
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    // 这里需要从黑板拿到的一些参数
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback>/*feedback*/)
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation whe the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // 这里定义了整个 tick 流程, tick 的实际操作在 on_tick 中
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // IDLE 状态下直接 on_tick 一次
      // user defined callback
      on_tick();

      // tick 完发送 goal 处理
      // 定义回调
      // 发送 goal
      // 定义 future_goal_handle_ 在未来获取 goal_handle_
      // 因为不想再一个地方一直等着服务拿到 goal, 所以先定义一个 shared_future,
      // 如果后续函数运行完成, 返回的东西会给 shared_future
      // 主要是发送完请求, 确保服务有响应, 并拿到 goal handle
      send_new_goal();
    }

    try {
      // 检查 future_goal_handle_ 是否存在, 服务是否响应了
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      if (future_goal_handle_) {
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          // 这是服务还没有响应, 等待时间没有超过 server_timeout_, 表示还在运行
          // return RUNNING if there is still some time before timeout happens
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // 如果服务花费太多时间了服务还没有反应
          // if server has taken more time than the specified timeout value return FAILURE
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // 如果 future_goal_handle_ 不存在, 被重置了, 表示 goal_handle_ 拿到了, 服务响应了
      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !goal_result_available_) {
        // 用户定义来处理反馈
        // user defined callback. May modify the value of "goal_updated_"
        on_wait_for_result(feedback_);

        // 处理完反馈后重置
        // reset feedback to avoid stale information
        feedback_.reset();

        // 检查状态, 如果 goal 更新了, 并且处于执行和接受状态, 重新发送 goal
        auto goal_status = goal_handle_->get_status();
        if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
        {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        callback_group_executor_.spin_some();

        // spin 等待拿结果
        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_) {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      if (e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server"))
      {
        // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;
      } else {
        // Internal exception to propogate to the tree
        throw e;
      }
    }

    // 最后判断结果执行的状态
    BT::NodeStatus status;
    // 这里拿到结果的回调可以获得 result
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    // 重置 goal_handle_, 准备下一个 goal
    goal_handle_.reset();
    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override
  {
    if (should_cancel_goal()) {
      // 拿个 future_cancel
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        // 这属于是没有成功 cancel
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    // cancel 之后回到 IDLE
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // 如果当前节点不在 RUNNING, 不需要 cancel
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // 都没有 goal_handle_, 意味着服务还没有拿到 goal, 不需要 cancel
    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // 只有 goal 被服务接受和执行了才应该 cancel
    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send new goal to action server
   */
  void send_new_goal()
  {
    // 最开始 result 不是 available 的
    goal_result_available_ = false;
    // 模板编程, typename 告诉编译器后面的是一个类型而不是静态成员
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    // 定义结果回调
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        if (future_goal_handle_) {
          // 如果 future_goal_handle_ 有东西, 而新的 result 来了
          // 这表示 future_goal_handle_ 没有被 reset
          RCLCPP_DEBUG(
            node_->get_logger(),
            "Goal result for %s available, but it hasn't received the goal response yet. "
            "It's probably a goal result for the last goal request", action_name_.c_str());
          return;
        }

        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          // 保存回调结果
          goal_result_available_ = true;
          result_ = result;
        }
      };
    // 定义反馈回调
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        feedback_ = feedback;
      };

    // 在未来的某个时间点获取目标句柄, 异步的
    // 如果 action_client_->async_send_goal(goal_, send_goal_options) 有返回
    // 那么 future_goal_handle_ 会拿到这个返回 typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr
    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // 这里如果等待时间超过了 server_timeout_, 就不等了
    // server has already timed out, no need to sleep
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    // 设置 timeout 为剩余时间和 bt 循环时间短的那一个
    auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    // 等待 timeout 时间来获取结果
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    // 因为这里又等待了 timeout 时间
    elapsed += timeout;

    // 如果结果是被打断
    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    // 如果结果是成功, 那么 future_goal_handle_ 拿到货给 goal_handle_
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      // 如果 goal_handle_ 没拿到货, 就是出问题了
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    return false;
  }

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    // 获取和累加恢复次数
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  // 发送 goal 的客户端
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // Goal 和 Result
  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // Feedback 运行中的状态反馈
  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // BT 中用来执行所有 operations 的 ros 节点
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // 发送新的 goal 时等待服务回应的时间
  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // BT 循环执行的一个循环的等待时间
  // The timeout value for BT loop execution
  std::chrono::milliseconds bt_loop_duration_;

  // 跟踪服务的状态
  // std::shared_future 是异步编程的一种工具, 它可以用来获取异步操作的结果
  // 与 std::future 相比, std::shared_future 的优点是它可以被多次访问, 而 std::future 只能访问一次
  // To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
  future_goal_handle_;
  rclcpp::Time time_goal_sent_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

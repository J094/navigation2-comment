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

#include <memory>

#include "nav2_util/node_thread.hpp"

namespace nav2_util
{

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  // 创建单线程执行器
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // 创建线程来执行代码
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      // 这个线程只执行一个节点的任务, spin() 结束后这个执行器会和节点断开
      executor_->add_node(node_);
      executor_->spin();
      executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  // 这里传入执行器了, 直接 spin
  thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

NodeThread::~NodeThread()
{
  executor_->cancel();
  thread_->join();
}

}  // namespace nav2_util

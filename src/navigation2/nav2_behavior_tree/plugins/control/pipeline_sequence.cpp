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

#include <stdexcept>
#include <sstream>
#include <string>

#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"

namespace nav2_behavior_tree
{

PipelineSequence::PipelineSequence(const std::string & name)
: BT::ControlNode(name, {})
{
}

PipelineSequence::PipelineSequence(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus PipelineSequence::tick()
{
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        ControlNode::haltChildren();
        last_child_ticked_ = 0;  // reset
        return status;
      case BT::NodeStatus::SUCCESS:
        // do nothing and continue on to the next child. If it is the last child
        // we'll exit the loop and hit the wrap-up code at the end of the method.
        break;
      case BT::NodeStatus::RUNNING:
        // 这里只允许没有 tick 过的节点返回状态
        if (i >= last_child_ticked_) {
          last_child_ticked_ = i;
          return status;
        }
        // else do nothing and continue on to the next child
        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status <<
          "from child " << children_nodes_[i]->name();
        throw std::runtime_error(error_msg.str());
    }
  }
  // 所有节点都 tick 过, 所有节点都 SUCCESS 过, 返回 SUCCESS
  // Wrap up.
  ControlNode::haltChildren();
  last_child_ticked_ = 0;  // reset
  return BT::NodeStatus::SUCCESS;
}

void PipelineSequence::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
}

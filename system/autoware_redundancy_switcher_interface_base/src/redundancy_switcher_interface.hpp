// Copyright 2025 The Autoware Contributors
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
#pragma once

#include "log_adapter.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "redundancy_switcher_interface_base/core_logic/processor.hpp"
#include "redundancy_switcher_interface_base/plugin/command_bus.hpp"
#include "redundancy_switcher_interface_base/plugin/event_gateway.hpp"
#include "redundancy_switcher_interface_base/plugin/i_adapter_plugin.hpp"

#include <memory>
#include <vector>

namespace redundancy_switcher
{

class RedundancySwitcherInterface : public rclcpp::Node
{
public:
  explicit RedundancySwitcherInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RedundancySwitcherInterface() override;

private:
  std::shared_ptr<Processor> processor_;
  std::shared_ptr<CommandBus> command_bus_;
  std::shared_ptr<EventGateway> gateway_;

  // 常時有効な組み込み Adapter（pluginlib 不要）
  std::shared_ptr<LogAdapter> log_adapter_;

  pluginlib::ClassLoader<IAdapterPlugin> switcher_plugin_loader_;
  pluginlib::ClassLoader<IAdapterPlugin> subsystem_plugin_loader_;

  std::shared_ptr<IAdapterPlugin> switcher_plugin_;
  std::shared_ptr<IAdapterPlugin> subsystem_plugin_;
};

}  // namespace redundancy_switcher

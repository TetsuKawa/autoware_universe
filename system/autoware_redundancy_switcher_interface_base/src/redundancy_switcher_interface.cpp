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
#include "redundancy_switcher_interface.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <memory>

namespace redundancy_switcher
{

RedundancySwitcherInterface::RedundancySwitcherInterface(const rclcpp::NodeOptions & options)
: Node("redundancy_switcher_interface", options),
  switcher_plugin_loader_(
    "autoware_redundancy_switcher_interface", "redundancy_switcher::IAdapterPlugin"),
  subsystem_plugin_loader_(
    "autoware_redundancy_switcher_interface", "redundancy_switcher::IAdapterPlugin")
{
  processor_ = std::make_shared<Processor>();
  command_bus_ = std::make_shared<CommandBus>();
  gateway_ = std::make_shared<EventGateway>(processor_, command_bus_);

  // LogAdapter は常時有効（pluginlib 不要、直接インスタンス化）
  log_adapter_ = std::make_shared<LogAdapter>();
  command_bus_->add_handler(log_adapter_);
  log_adapter_->initialize(this, gateway_);

  this->declare_parameter("switcher_plugin", rclcpp::ParameterType::PARAMETER_STRING);
  this->declare_parameter("subsystem_plugin", rclcpp::ParameterType::PARAMETER_STRING);
  const auto switcher_plugin_name = this->get_parameter("switcher_plugin").as_string();
  const auto subsystem_plugin_name = this->get_parameter("subsystem_plugin").as_string();

  switcher_plugin_ = switcher_plugin_loader_.createSharedInstance(switcher_plugin_name);
  command_bus_->add_handler(switcher_plugin_);
  switcher_plugin_->initialize(this, gateway_);

  subsystem_plugin_ = subsystem_plugin_loader_.createSharedInstance(subsystem_plugin_name);
  command_bus_->add_handler(subsystem_plugin_);
  subsystem_plugin_->initialize(this, gateway_);
}

RedundancySwitcherInterface::~RedundancySwitcherInterface() = default;

}  // namespace redundancy_switcher

RCLCPP_COMPONENTS_REGISTER_NODE(redundancy_switcher::RedundancySwitcherInterface)

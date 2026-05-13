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
    "autoware_redundancy_switcher_interface", "redundancy_switcher::IAdapterPlugin")
{
  const bool is_redundant = this->declare_parameter<bool>("is_redundant", true);

  processor_ = std::make_shared<Processor>();
  command_bus_ = std::make_shared<CommandBus>();
  gateway_ = std::make_shared<EventGateway>(processor_, command_bus_);

  // LogAdapter は常時有効（pluginlib 不要、直接インスタンス化）
  log_adapter_ = std::make_shared<LogAdapter>();
  command_bus_->add_handler(log_adapter_);
  log_adapter_->initialize(this, gateway_);

  // DiagAdapter は常時有効（pluginlib 不要、直接インスタンス化）
  diag_adapter_ = std::make_shared<DiagAdapter>();
  command_bus_->add_handler(diag_adapter_);
  diag_adapter_->initialize(this, gateway_);

  // SubSystemAdapter は常時有効（pluginlib 不要、直接インスタンス化）
  subsystem_adapter_ = std::make_shared<SubSystemAdapter>();
  command_bus_->add_handler(subsystem_adapter_);
  subsystem_adapter_->initialize(this, gateway_);

  if (is_redundant) {
    this->declare_parameter("switcher_plugin", rclcpp::ParameterType::PARAMETER_STRING);
    const auto switcher_plugin_name = this->get_parameter("switcher_plugin").as_string();
    switcher_plugin_ = switcher_plugin_loader_.createSharedInstance(switcher_plugin_name);
    command_bus_->add_handler(switcher_plugin_);
    switcher_plugin_->initialize(this, gateway_);
  } else {
    // 非冗長システム: switcher_plugin をロードせず、常に stable 状態を注入する。
    // Processor から見て is_stable=true の SwitcherSignals を受け取っている状態となり、
    // - diag: OK
    // - reset: SUCCESS "not necessary"
    // と動作する。
    gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
      Annotated<SwitcherSignals>{{true, false, false}, "non-redundant system"}}});
  }
}

RedundancySwitcherInterface::~RedundancySwitcherInterface() = default;

}  // namespace redundancy_switcher

RCLCPP_COMPONENTS_REGISTER_NODE(redundancy_switcher::RedundancySwitcherInterface)

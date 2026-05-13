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
#include "switcher_adapter.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <memory>
#include <stdexcept>

namespace redundancy_switcher
{

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

void SwitcherAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) {
    throw std::invalid_argument("SwitcherAdapter: node is null");
  }
  if (!gateway) {
    throw std::invalid_argument("SwitcherAdapter: gateway is null");
  }
  node_ = node;
  gateway_ = gateway;
}

void SwitcherAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [](const SelfInterruptionCommand &) { /* TODO: send self interruption to switcher */ },
      [](const ResetCommand &) { /* TODO: send reset to switcher */ }, [](const auto &) {}},
    command);
}

}  // namespace redundancy_switcher

PLUGINLIB_EXPORT_CLASS(redundancy_switcher::SwitcherAdapter, redundancy_switcher::IAdapterPlugin)

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

#include "node.hpp"

namespace autoware::redundancy_service_divider
{

RedundancyServiceDivider::RedundancyServiceDivider(const rclcpp::NodeOptions & node_options)
: Node("redundancy_service_divider", node_options)
{
  using ResetRedundancySwitcher = tier4_system_msgs::srv::ResetRedundancySwitcher;

  const int  timeout      = static_cast<int>(declare_parameter("service_timeout_ms", 200));
  const bool is_redundant = declare_parameter("is_redundant", true);

  // Create once and share between divider and ready_divider to avoid duplicate clients.
  auto reset_pair = make_client_pair<ResetRedundancySwitcher>(*this, "reset_redundancy_switcher", is_redundant);

  divider_       = std::make_unique<ServiceDivider>(*this, timeout, is_redundant, reset_pair);
  ready_divider_ = std::make_unique<AutowareReadyDivider>(*this, timeout, is_redundant, reset_pair);
}

}  // namespace autoware::redundancy_service_divider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_service_divider::RedundancyServiceDivider)

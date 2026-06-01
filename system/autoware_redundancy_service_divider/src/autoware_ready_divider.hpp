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

#ifndef AUTOWARE_READY_DIVIDER_HPP_
#define AUTOWARE_READY_DIVIDER_HPP_

#include "client_pair.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_system_msgs/srv/reset_redundancy_switcher.hpp>

#include <mutex>

namespace autoware::redundancy_service_divider
{

// Monitors Autoware initialization state and divides reset/init requests
// to both main and sub ECUs at the appropriate timing.
class AutowareReadyDivider
{
public:
  // reset_switcher_pair is shared with ServiceDivider to avoid duplicate client objects.
  AutowareReadyDivider(
    rclcpp::Node & node,
    int service_timeout_ms,
    bool is_redundant,
    ClientPair<tier4_system_msgs::srv::ResetRedundancySwitcher> reset_switcher_pair);

private:
  using LocalizationState       = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using RouteState              = autoware_adapi_v1_msgs::msg::RouteState;
  using OperationMode           = autoware_adapi_v1_msgs::msg::OperationModeState;
  using SetBool                 = std_srvs::srv::SetBool;
  using ResetRedundancySwitcher = tier4_system_msgs::srv::ResetRedundancySwitcher;

  enum class InitState {
    WAIT_SERVICES_READY,
    SET_AGGREGATOR_INIT,
    RESET_SWITCHER,
    SET_SWITCHER_IFACE_INIT,
    DONE,
  };

  rclcpp::Node & node_;
  int service_timeout_ms_;
  bool is_redundant_;

  // Protects all state members. Service call helpers block inside but do not
  // re-enter this class, so holding the lock during them is safe.
  mutable std::mutex state_mutex_;

  // Subscriptions
  rclcpp::Subscription<LocalizationState>::SharedPtr sub_localization_initialization_state_;
  rclcpp::Subscription<RouteState>::SharedPtr        sub_route_state_;
  rclcpp::Subscription<OperationMode>::SharedPtr     sub_operation_mode_state_;

  // Service client pairs (main / sub)
  ClientPair<SetBool>                 clients_set_aggregator_initializing_;
  ClientPair<ResetRedundancySwitcher> clients_reset_redundancy_switcher_;
  ClientPair<SetBool>                 clients_set_redundancy_switcher_interface_initializing_;

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr reset_redundancy_switcher_timer_;

  // State — all guarded by state_mutex_
  InitState init_state_{InitState::WAIT_SERVICES_READY};
  bool is_aggregator_initializing_{false};
  bool is_redundancy_switcher_interface_initializing_{false};
  LocalizationState::ConstSharedPtr localization_initialization_state_ptr_;
  RouteState::ConstSharedPtr        route_state_ptr_;
  OperationMode::ConstSharedPtr     operation_mode_state_ptr_;

  void advance_init_state();
  void apply_ready_state();  // must be called with state_mutex_ held
  bool set_aggregator_initializing(bool initializing);
  bool reset_redundancy_switcher();
  bool set_redundancy_switcher_interface_initializing(bool initializing);
  bool is_autoware_ready() const;
  bool is_initializing() const;
};

}  // namespace autoware::redundancy_service_divider

#endif  // AUTOWARE_READY_DIVIDER_HPP_

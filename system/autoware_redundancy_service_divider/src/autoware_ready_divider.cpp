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

#include "autoware_ready_divider.hpp"

#include <chrono>
#include <memory>
#include <mutex>

namespace autoware::redundancy_service_divider
{

AutowareReadyDivider::AutowareReadyDivider(
  rclcpp::Node & node,
  int service_timeout_ms,
  bool is_redundant,
  ClientPair<ResetRedundancySwitcher> reset_switcher_pair)
: node_(node),
  service_timeout_ms_(service_timeout_ms),
  is_redundant_(is_redundant),
  clients_reset_redundancy_switcher_(reset_switcher_pair)
{
  const auto qos = rclcpp::QoS(1).transient_local();

  sub_localization_initialization_state_ = node_.create_subscription<LocalizationState>(
    "~/input/localization_initialization_state", qos,
    [this](const LocalizationState::ConstSharedPtr & msg) {
      std::lock_guard lock(state_mutex_);
      localization_initialization_state_ptr_ = msg;
      apply_ready_state();
    });
  sub_route_state_ = node_.create_subscription<RouteState>(
    "~/input/route_state", qos,
    [this](const RouteState::ConstSharedPtr & msg) {
      std::lock_guard lock(state_mutex_);
      route_state_ptr_ = msg;
      apply_ready_state();
    });
  sub_operation_mode_state_ = node_.create_subscription<OperationMode>(
    "~/input/operation_mode_state", qos,
    [this](const OperationMode::ConstSharedPtr & msg) {
      std::lock_guard lock(state_mutex_);
      operation_mode_state_ptr_ = msg;
      apply_ready_state();
    });

  clients_set_aggregator_initializing_                    = make_client_pair<SetBool>(node_, "set_aggregator_initializing", is_redundant_);
  clients_set_redundancy_switcher_interface_initializing_ = make_client_pair<SetBool>(node_, "set_redundancy_switcher_interface_initializing", is_redundant_);

  init_timer_ = rclcpp::create_timer(
    &node_, node_.get_clock(), std::chrono::seconds(1),
    [this]() { advance_init_state(); });
}

void AutowareReadyDivider::advance_init_state()
{
  std::lock_guard lock(state_mutex_);

  const auto services_ready = [this](const auto & pair) {
    return pair.main->service_is_ready() && (!is_redundant_ || pair.sub->service_is_ready());
  };

  switch (init_state_) {
    case InitState::WAIT_SERVICES_READY:
      if (!services_ready(clients_set_aggregator_initializing_)) {
        RCLCPP_INFO(node_.get_logger(), "Waiting for set_aggregator_initializing service");
        return;
      }
      if (is_redundant_ && !services_ready(clients_reset_redundancy_switcher_)) {
        RCLCPP_INFO(node_.get_logger(), "Waiting for reset_redundancy_switcher service");
        return;
      }
      if (!services_ready(clients_set_redundancy_switcher_interface_initializing_)) {
        RCLCPP_INFO(node_.get_logger(), "Waiting for set_redundancy_switcher_interface_initializing service");
        return;
      }
      init_state_ = InitState::SET_AGGREGATOR_INIT;
      [[fallthrough]];

    case InitState::SET_AGGREGATOR_INIT:
      if (!set_aggregator_initializing(true)) return;
      init_state_ = InitState::RESET_SWITCHER;
      [[fallthrough]];

    case InitState::RESET_SWITCHER:
      if (!reset_redundancy_switcher()) return;
      init_state_ = InitState::SET_SWITCHER_IFACE_INIT;
      [[fallthrough]];

    case InitState::SET_SWITCHER_IFACE_INIT:
      if (!set_redundancy_switcher_interface_initializing(true)) return;
      init_state_ = InitState::DONE;
      init_timer_->cancel();
      reset_redundancy_switcher_timer_ = rclcpp::create_timer(
        &node_, node_.get_clock(), std::chrono::seconds(5),
        [this]() {
          std::lock_guard lock(state_mutex_);
          if (!is_initializing()) return;
          // If Autoware is ready, retry the full sequence to clear initializing flags.
          // Otherwise keep the redundancy switcher reset until Autoware becomes ready.
          if (is_autoware_ready()) {
            apply_ready_state();
          } else if (!reset_redundancy_switcher()) {
            RCLCPP_WARN(node_.get_logger(), "Periodic reset_redundancy_switcher failed");
          }
        });
      // Autoware may already be ready; check without waiting for the next message.
      apply_ready_state();
      break;

    case InitState::DONE:
      break;
  }
}

void AutowareReadyDivider::apply_ready_state()
{
  if (init_state_ != InitState::DONE) return;
  if (!is_autoware_ready()) return;
  if (!is_initializing()) return;

  const bool localized =
    localization_initialization_state_ptr_->state == LocalizationState::INITIALIZED;
  const bool route_set        = route_state_ptr_->state == RouteState::SET;
  const bool autoware_control = operation_mode_state_ptr_->is_autoware_control_enabled;

  if (localized && route_set && autoware_control) {
    if (!set_aggregator_initializing(false)) return;
    if (!reset_redundancy_switcher()) return;
    if (!set_redundancy_switcher_interface_initializing(false)) return;
  }
}

bool AutowareReadyDivider::set_aggregator_initializing(bool initializing)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = initializing;
  if (!call_client_pair(
        clients_set_aggregator_initializing_, request,
        "set_aggregator_initializing", service_timeout_ms_, node_.get_logger(),
        [](auto r) { return r->success; }, is_redundant_))
    return false;
  is_aggregator_initializing_ = initializing;
  return true;
}

bool AutowareReadyDivider::reset_redundancy_switcher()
{
  if (!is_redundant_) return true;
  auto request = std::make_shared<ResetRedundancySwitcher::Request>();
  return call_client_pair(
    clients_reset_redundancy_switcher_, request,
    "reset_redundancy_switcher", service_timeout_ms_, node_.get_logger(),
    [](auto r) { return r->status.success; });
}

bool AutowareReadyDivider::set_redundancy_switcher_interface_initializing(bool initializing)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = initializing;
  if (!call_client_pair(
        clients_set_redundancy_switcher_interface_initializing_, request,
        "set_redundancy_switcher_interface_initializing", service_timeout_ms_, node_.get_logger(),
        [](auto r) { return r->success; }, is_redundant_))
    return false;
  is_redundancy_switcher_interface_initializing_ = initializing;
  return true;
}

bool AutowareReadyDivider::is_autoware_ready() const
{
  return localization_initialization_state_ptr_ &&
         route_state_ptr_ &&
         operation_mode_state_ptr_;
}

bool AutowareReadyDivider::is_initializing() const
{
  return is_aggregator_initializing_ || is_redundancy_switcher_interface_initializing_;
}

}  // namespace autoware::redundancy_service_divider

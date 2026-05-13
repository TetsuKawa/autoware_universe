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

#include "uds_receiver.hpp"
#include "uds_sender.hpp"
#include "uds_types.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

namespace redundancy_switcher
{

class SwitcherAdapter : public IAdapterPlugin
{
public:
  SwitcherAdapter() = default;
  ~SwitcherAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  void on_switcher_status(const ElectionStatus & status);
  void check_election_status_timeout();
  void check_switcher_connection();
  bool no_data(diagnostic_updater::DiagnosticStatusWrapper & stat) const;

  static std::string node_state_to_string(uint8_t node_state);
  static SwitcherSignals to_switcher_signals(uint8_t node_state);

  // ── 診断コールバック ─────────────────────────────────────────────────
  void update_switcher_connection_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_node_state_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_vcu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_vcu_fault_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_sub_ecu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_main_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_ecu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_to_main_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_sub_ecu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void update_main_vcu_to_sub_vcu_link_fault_diag(
    diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;

  // UDS: Interface → Switcher (コマンド送信)
  std::unique_ptr<UdsSender<ElectionRequest>> uds_sender_;
  // UDS: Switcher → Interface (ステータス受信)
  std::unique_ptr<UdsReceiver<ElectionStatus>> uds_receiver_;

  // Parameters
  double election_status_timeout_milli_{1000.0};
  bool is_main_ecu_{true};

  // State
  std::optional<ElectionStatus> last_election_status_;
  std::optional<rclcpp::Time> stamp_election_status_;
  std::optional<AutowareReady> autoware_ready_;  // UpdateAutowareReadyCommand でキャッシュ
  bool another_ecu_availability_timeout_{false};  // UpdateAnotherEcuAvailabilityTimeoutCommand でキャッシュ
  std::unordered_set<std::string> node_fault_points_;
  std::unordered_set<std::string> link_fault_points_;

  // Timers
  rclcpp::TimerBase::SharedPtr timeout_check_timer_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace redundancy_switcher

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

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface_base/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface_base/plugin/i_adapter_plugin.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_external_api_msgs/srv/reset_redundancy_switcher.hpp>
#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tmp_msgs/msg/active_control_unit.hpp>

#include <memory>
#include <optional>
#include <string>

namespace redundancy_switcher
{

using CommandModeRequest = tier4_system_msgs::msg::CommandModeRequest;
using CommandModeAvailability = tier4_system_msgs::msg::CommandModeAvailability;
using ActiveControlUnit = tmp_msgs::msg::ActiveControlUnit;
using SetBool = std_srvs::srv::SetBool;
using ResetRedundancySwitcher = tier4_external_api_msgs::srv::ResetRedundancySwitcher;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;
using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;

/**
 * @brief 下位系 (Autoware / SubSystem) との I/O を担う ROS Adapter
 *
 * 責任:
 *   - Autoware からの ROS メッセージを InputEvent に変換して Processor へ投入
 *   - Processor が返した OutputCommands を CommandBus 経由で配信
 *   - SendSubsystemCommand / PublishSubsystemStatusCommand のみ execute() で処理
 *   - 非同期管理（タイムアウト監視、診断更新）
 */
class SubSystemAdapter : public IAdapterPlugin
{
public:
  SubSystemAdapter() = default;
  ~SubSystemAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;

  void execute(const OutputCommand & command) override;

private:
  // ── イベント変換・投入ヘルパ ────────────────────────────────────────────
  void submit_event(const InputEvent & event);

  // ── ROS コールバック ────────────────────────────────────────────────────
  void on_operation_mode_state(const OperationModeState::ConstSharedPtr msg);
  void on_command_mode_request(const CommandModeRequest::ConstSharedPtr msg);
  void on_command_mode_availability(const CommandModeAvailability::ConstSharedPtr msg);
  void on_set_initializing(
    const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response);
  void on_reset_request(
    const ResetRedundancySwitcher::Request::SharedPtr request,
    ResetRedundancySwitcher::Response::SharedPtr response);

  void on_velocity_report(const VelocityReport::ConstSharedPtr msg);
  void on_control_mode_report(const ControlModeReport::ConstSharedPtr msg);

  // ── Sub ECU ERROR
  void check_sub_ecu_error(const CommandModeAvailability::ConstSharedPtr msg);

  // ── タイムアウト監視 ────────────────────────────────────────────────────
  void check_availability_timeout(const CommandModeAvailability::ConstSharedPtr msg);

  // ── エフェクト実行 ──────────────────────────────────────────────────────
  void send_active_control_unit(const UpdateActiveControlUnitCommand & command);

  // ── 診断 ───────────────────────────────────────────────────────────────
  void update_redundancy_switcher_status_diag(DiagStatusWrapper & stat);
  void update_main_ecu_fault_diag(DiagStatusWrapper & stat);
  void update_sub_ecu_fault_diag(DiagStatusWrapper & stat);
  void update_main_vcu_fault_diag(DiagStatusWrapper & stat);
  void update_sub_vcu_fault_diag(DiagStatusWrapper & stat);
  void update_main_ecu_to_sub_ecu_link_fault_diag(DiagStatusWrapper & stat);
  void update_main_ecu_to_main_vcu_link_fault_diag(DiagStatusWrapper & stat);
  void update_main_ecu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat);
  void update_sub_ecu_to_main_vcu_link_fault_diag(DiagStatusWrapper & stat);
  void update_sub_ecu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat);
  void update_main_vcu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat);

  /// notification_diags から指定キーのレベルを取得し、OK/ERROR で stat を更新する。
  /// テスト容易性のため gateway_ のみに依存し、ROS 型には依存しない。
  void update_fault_diag(
    DiagStatusWrapper & stat, const std::string & key, const std::string & healthy_msg,
    const std::string & fault_msg) const;

  // ── ユーティリティ ─────────────────────────────────────────────────────

  // inline
  inline auto to_ros_level(DiagLevelIR level)
  {
    switch (level) {
      case DiagLevelIR::Ok:
        return DiagnosticStatus::OK;
      case DiagLevelIR::Warn:
        return DiagnosticStatus::WARN;
      case DiagLevelIR::Error:
        return DiagnosticStatus::ERROR;
      case DiagLevelIR::Stale:
        return DiagnosticStatus::STALE;
      default:
        return DiagnosticStatus::ERROR;
    }
  }

  // ── メンバ変数 ─────────────────────────────────────────────────────────
  rclcpp::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;

  // Parameters
  bool is_main_ecu_{true};
  double availability_timeout_milli_{1.0};

  // State
  std::optional<CommandModeRequest> last_command_mode_request_;
  std::optional<rclcpp::Time> stamp_another_ecu_availability_;
  bool is_another_ecu_availability_timeout_{false};

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  // Publishers
  rclcpp::Publisher<ActiveControlUnit>::SharedPtr pub_active_control_unit_;

  // Subscribers
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<CommandModeRequest>::SharedPtr sub_command_mode_request_;
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_command_mode_availability_;

  // Services
  rclcpp::Service<SetBool>::SharedPtr srv_set_initializing_;
  rclcpp::Service<ResetRedundancySwitcher>::SharedPtr srv_reset_;
};

}  // namespace redundancy_switcher

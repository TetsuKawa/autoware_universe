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
#include "subsystem_adapter.hpp"

#include <autoware_command_mode_types/modes.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// overloaded helper (C++17)
// ---------------------------------------------------------------------------

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------

void SubSystemAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) {
    throw std::invalid_argument("SubSystemAdapter: node is null");
  }
  if (!gateway) {
    throw std::invalid_argument("SubSystemAdapter: gateway is null");
  }

  node_ = node;
  gateway_ = gateway;

  is_main_ecu_ = node_->declare_parameter<bool>("is_main_ecu", true);
  availability_timeout_milli_ = node_->declare_parameter<double>("availability_timeout_milli", 1.0);

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID(
    is_main_ecu_ ? "main_ecu_redundancy_switcher_interface"
                 : "sub_ecu_redundancy_switcher_interface");
  updater_->add(
    "redundancy_switcher_status", this, &SubSystemAdapter::update_redundancy_switcher_status_diag);
  updater_->add("main_ecu_fault", this, &SubSystemAdapter::update_main_ecu_fault_diag);
  updater_->add("sub_ecu_fault", this, &SubSystemAdapter::update_sub_ecu_fault_diag);
  updater_->add("main_vcu_fault", this, &SubSystemAdapter::update_main_vcu_fault_diag);
  updater_->add("sub_vcu_fault", this, &SubSystemAdapter::update_sub_vcu_fault_diag);
  updater_->add(
    "main_ecu_to_sub_ecu_link_fault", this,
    &SubSystemAdapter::update_main_ecu_to_sub_ecu_link_fault_diag);
  updater_->add(
    "main_ecu_to_main_vcu_link_fault", this,
    &SubSystemAdapter::update_main_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "main_ecu_to_sub_vcu_link_fault", this,
    &SubSystemAdapter::update_main_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_main_vcu_link_fault", this,
    &SubSystemAdapter::update_sub_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_sub_vcu_link_fault", this,
    &SubSystemAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "main_vcu_to_sub_vcu_link_fault", this,
    &SubSystemAdapter::update_main_vcu_to_sub_vcu_link_fault_diag);

  const auto qos = rclcpp::QoS(1);

  sub_velocity_report_ = node_->create_subscription<VelocityReport>(
    "~/input/velocity", qos,
    std::bind(&SubSystemAdapter::on_velocity_report, this, std::placeholders::_1));

  sub_control_mode_ = node_->create_subscription<ControlModeReport>(
    "~/input/control_mode", qos,
    std::bind(&SubSystemAdapter::on_control_mode_report, this, std::placeholders::_1));

  sub_operation_mode_state_ = node_->create_subscription<OperationModeState>(
    "~/input/operation_mode_state", qos,
    std::bind(&SubSystemAdapter::on_operation_mode_state, this, std::placeholders::_1));

  sub_command_mode_request_ = node_->create_subscription<CommandModeRequest>(
    "~/input/command_mode_request", qos,
    std::bind(&SubSystemAdapter::on_command_mode_request, this, std::placeholders::_1));

  sub_command_mode_availability_ = node_->create_subscription<CommandModeAvailability>(
    "~/input/command_mode_availability", qos,
    std::bind(&SubSystemAdapter::on_command_mode_availability, this, std::placeholders::_1));

  srv_set_initializing_ = node_->create_service<SetBool>(
    "~/set_initializing",
    std::bind(
      &SubSystemAdapter::on_set_initializing, this, std::placeholders::_1, std::placeholders::_2));

  srv_reset_ = node_->create_service<ResetRedundancySwitcher>(
    "~/service/reset",
    std::bind(
      &SubSystemAdapter::on_reset_request, this, std::placeholders::_1, std::placeholders::_2));

  // Publishers
  pub_active_control_unit_ =
    node_->create_publisher<ActiveControlUnit>("~/output/active_control_unit", qos);
}

// ---------------------------------------------------------------------------
// submit_event — Processor に投入 → CommandBus へ配信
// ---------------------------------------------------------------------------

void SubSystemAdapter::submit_event(const InputEvent & event)
{
  gateway_->submit(event);  // スレッドセーフ: handle() + dispatch() をアトミックに実行
}

// ---------------------------------------------------------------------------
// Inbound ROS callbacks
// ---------------------------------------------------------------------------

void SubSystemAdapter::on_operation_mode_state(OperationModeState::ConstSharedPtr msg)
{
  (void)msg;
  // TODO(autoware): 実装
}

void SubSystemAdapter::on_command_mode_request(const CommandModeRequest::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;

  // main ECU のみ処理。空メッセージは無視。
  if (!is_main_ecu_ || msg->items.empty()) return;

  // std::exchange でアトミックに旧値取得＆更新。初回または変化なしはスキップ。
  const auto prev = std::exchange(last_command_mode_request_, *msg);
  if (!prev.has_value() || prev->items == msg->items) return;

  const auto & mode = msg->items[0].mode;
  if (mode == modes::sub_ecu_standby || mode == modes::sub_ecu_in_lane_moderate_stop) {
    submit_event(InputEvent{SelfInterruptionEvent{}});
  }
}

void SubSystemAdapter::on_command_mode_availability(
  const CommandModeAvailability::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;
  // SubECU の異常検知
  check_sub_ecu_error(msg);
  check_availability_timeout(msg);
}

void SubSystemAdapter::check_sub_ecu_error(const CommandModeAvailability::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;

  if (!is_main_ecu_) return;

  for (const auto & item : msg->items) {
    if (item.mode == modes::sub_ecu_in_lane_moderate_stop && !item.available) {
      submit_event(InputEvent{SelfInterruptionEvent{}});
      return;
    }
  }
}

void SubSystemAdapter::check_availability_timeout(const CommandModeAvailability::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;

  // 相手 ECU のアイテムが含まれるか判定（is_main_ecu_ に応じて対象モードが変わる）
  const bool has_other_ecu_items =
    std::any_of(msg->items.begin(), msg->items.end(), [&](const auto & item) {
      if (is_main_ecu_) {
        return item.mode == modes::sub_ecu_in_lane_moderate_stop ||
               item.mode == modes::sub_ecu_standby;
      }
      return item.mode == modes::comfortable_stop ||
             item.mode == modes::main_ecu_in_lane_moderate_stop ||
             item.mode == modes::main_ecu_in_lane_emergency_stop;
    });

  // スタンプ更新前の値でタイムアウト判定（メッセージ間隔を測定）
  const auto prev_stamp = stamp_another_ecu_availability_;
  if (has_other_ecu_items) stamp_another_ecu_availability_ = node_->now();

  if (prev_stamp.has_value()) {
    const double elapsed_ms = (node_->now() - *prev_stamp).seconds() * 1000.0;
    is_another_ecu_availability_timeout_ = elapsed_ms > availability_timeout_milli_;
  }
  // TODO(kawaguchi): タイムアウト時のイベント投入を実装
}

void SubSystemAdapter::on_set_initializing(
  const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response)
{
  submit_event(InputEvent{SetAutowareReadyEvent{!request->data}});

  // TODO(autoware): Processor の状態に応じて適切なレスポンスを返すようにする
  response->success = true;
  response->message = "Set initializing: " + std::string(request->data ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
}

void SubSystemAdapter::on_reset_request(
  const ResetRedundancySwitcher::Request::SharedPtr request [[maybe_unused]],
  ResetRedundancySwitcher::Response::SharedPtr response)
{
  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

  submit_event(InputEvent{ResetEvent{}});

  response->status.code = ResponseStatus::SUCCESS;
  response->status.message = "Reset request accepted.";
  RCLCPP_INFO(node_->get_logger(), "Reset request accepted.");
}

void SubSystemAdapter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  constexpr auto th_stopped_velocity = 0.001;
  const bool is_stopped = std::abs(msg->longitudinal_velocity) < th_stopped_velocity;
  submit_event(InputEvent{SetVehicleStoppedEvent{is_stopped}});
}

void SubSystemAdapter::on_control_mode_report(const ControlModeReport::ConstSharedPtr msg)
{
  const bool is_autoware_control = (msg->mode == ControlModeReport::AUTONOMOUS);
  submit_event(InputEvent{SetAutowareControlEvent{is_autoware_control}});
}

void SubSystemAdapter::update_redundancy_switcher_status_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto snap = gateway_->snapshot();
  const auto merged = snap.interface_diags + snap.switcher_diags;

  auto worst = DiagLevelIR::Ok;
  for (const auto & [key, value] : merged.values) {
    stat.add(key, value.level_string() + ": " + value.message);
    worst = std::max(worst, value.level);
  }

  const auto to_msg = [](DiagLevelIR level) -> const char * {
    switch (level) {
      case DiagLevelIR::Ok:
        return "redundancy switcher running";
      case DiagLevelIR::Warn:
        return "redundancy switcher has warnings";
      case DiagLevelIR::Error:
        return "redundancy switcher has errors";
      default:
        return "redundancy switcher data is stale";
    }
  };
  stat.summary(to_ros_level(worst), to_msg(worst));
}

// ---------------------------------------------------------------------------
// update_fault_diag — 個別 fault diag の共通実装
// notification_diags から key を検索し、OK/ERROR で stat を更新する。
// key が存在しない場合は STALE を返す（データ未受信）。
// ---------------------------------------------------------------------------

void SubSystemAdapter::update_fault_diag(
  DiagStatusWrapper & stat, const std::string & key, const std::string & healthy_msg,
  const std::string & fault_msg) const
{
  const auto & diags = gateway_->snapshot().notification_diags;
  const auto it = diags.values.find(key);
  if (it == diags.values.end()) {
    stat.summary(DiagnosticStatus::STALE, key + ": no data received");
    return;
  }
  const bool is_ok = (it->second.level == DiagLevelIR::Ok);
  stat.summary(
    is_ok ? DiagnosticStatus::OK : DiagnosticStatus::ERROR, is_ok ? healthy_msg : fault_msg);
}

// TODO(autoware):
// v4.3.0の通知方式に依存しているため、redundancy_switcher側を知っている前提で実装している。将来的には、Processorの状態をGateway経由で取得して診断する形にリファクタリングする。
void SubSystemAdapter::update_main_ecu_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(stat, "main_ecu", "Main ECU is healthy", "Main ECU fault detected");
}
void SubSystemAdapter::update_sub_ecu_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(stat, "sub_ecu", "Sub ECU is healthy", "Sub ECU fault detected");
}
void SubSystemAdapter::update_main_vcu_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(stat, "main_vcu", "Main VCU is healthy", "Main VCU fault detected");
}
void SubSystemAdapter::update_sub_vcu_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(stat, "sub_vcu", "Sub VCU is healthy", "Sub VCU fault detected");
}
void SubSystemAdapter::update_main_ecu_to_sub_ecu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "main_ecu_to_sub_ecu_link", "Main-Sub ECU link is healthy",
    "Main-Sub ECU link fault detected");
}
void SubSystemAdapter::update_main_ecu_to_main_vcu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "main_ecu_to_main_vcu_link", "Main ECU to Main VCU link is healthy",
    "Main ECU to Main VCU link fault detected");
}
void SubSystemAdapter::update_main_ecu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "main_ecu_to_sub_vcu_link", "Main ECU to Sub VCU link is healthy",
    "Main ECU to Sub VCU link fault detected");
}
void SubSystemAdapter::update_sub_ecu_to_main_vcu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "sub_ecu_to_main_vcu_link", "Sub ECU to Main VCU link is healthy",
    "Sub ECU to Main VCU link fault detected");
}
void SubSystemAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "sub_ecu_to_sub_vcu_link", "Sub ECU to Sub VCU link is healthy",
    "Sub ECU to Sub VCU link fault detected");
}
void SubSystemAdapter::update_main_vcu_to_sub_vcu_link_fault_diag(DiagStatusWrapper & stat)
{
  update_fault_diag(
    stat, "main_vcu_to_sub_vcu_link", "Main-Sub VCU link is healthy",
    "Main-Sub VCU link fault detected");
}

// ---------------------------------------------------------------------------
// Outbound: OutputCommand の処理（担当分のみ）
// ---------------------------------------------------------------------------

void SubSystemAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const UpdateIntefaceDiagCommand &) { updater_->force_update(); },
      [this](const UpdateSwitcherDiagCommand &) { updater_->force_update(); },
      [this](const UpdateNotificationDiagCommand &) { updater_->force_update(); },
      [this](const UpdateActiveControlUnitCommand & e) { send_active_control_unit(e); },
      [](const auto &) { /* 他 Adapter の担当 — 無視 */ }},
    command);
}

void SubSystemAdapter::send_active_control_unit(const UpdateActiveControlUnitCommand & command)
{
  ActiveControlUnit msg;
  msg.ids = command.unit_ids;
  pub_active_control_unit_->publish(msg);
}

}  // namespace redundancy_switcher

PLUGINLIB_EXPORT_CLASS(redundancy_switcher::SubSystemAdapter, redundancy_switcher::IAdapterPlugin)

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
#include "diag_adapter.hpp"

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

void DiagAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> /*gateway*/)
{
  if (!node) throw std::invalid_argument("DiagAdapter: node is null");

  node_ = node;
  transitional_timeout_milli_ =
    node->declare_parameter<double>("diag.transitional_timeout_milli", 10000.0);

  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID("redundancy_switcher_interface");
  updater_->add(
    "redundancy_switcher_interface/status", this, &DiagAdapter::update_status);
}

// ---------------------------------------------------------------------------
// execute — UpdateStatusDiagCommand のみ処理し、他はすべて無視
// ---------------------------------------------------------------------------

void DiagAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const UpdateStatusDiagCommand & cmd) {
        last_snapshot_ = cmd.snapshot;  // コマンドに埋め込まれた snapshot をキャッシュ
        updater_->force_update();
      },
      [](const auto &) { /* 他 Adapter の担当 — 無視 */ }},
    command);
}

// ---------------------------------------------------------------------------
// update_status — DomainSnapshot の enum 4 状態を集約して publish
// ---------------------------------------------------------------------------

void DiagAdapter::update_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  // last_snapshot_ が未設定 = UpdateStatusDiagCommand をまだ受け取っていない（起動直後）
  if (!last_snapshot_) {
    stat.summary(DiagStatus::WARN, "Startup not yet complete: awaiting first status update");
    return;
  }
  const auto & snap = *last_snapshot_;

  // nullopt = 未受信（起動準備未完了）→ WARN "Startup not yet complete"
  // 各フィールドは nullopt のときは WARN を返し、それ以外は enum 値に応じて判定する。

  // ── SwitcherSignals ──────────────────────────────────────────────────
  // 旧実装の update_diagnostics と同等の判定をここで行う:
  //   nullopt (timeout)      → ERROR  (旧: election_status_timeout 超過 → ERROR)
  //   is_faulted             → ERROR  (旧: ELECTION_UNCLOSED/PATH_NOT_FOUND → ERROR)
  //   is_self_interrupted    → WARN
  //   is_stable              → OK
  //   過渡状態 + タイムアウト → ERROR  (旧: initializing_timeout 超過 → ERROR)
  //   過渡状態 (タイムアウト前) → WARN
  const auto now = node_->now();
  auto switcher_level = DiagStatus::OK;
  std::string switcher_msg;

  if (!snap.switcher.has_value()) {
    switcher_level = DiagStatus::WARN;
    switcher_msg = "Startup not yet complete: awaiting switcher data";
    stamp_transitional_start_ = std::nullopt;
  } else {
    const auto & sw = *snap.switcher;
    if (sw.value.is_faulted) {
      switcher_level = DiagStatus::ERROR;
      switcher_msg = "Switcher fault: " + sw.annotation;
      stamp_transitional_start_ = std::nullopt;
    } else if (sw.value.is_self_interrupted) {
      switcher_level = DiagStatus::WARN;
      switcher_msg = "Self-interruption occurred: " + sw.annotation;
      stamp_transitional_start_ = std::nullopt;
    } else if (sw.value.is_stable) {
      switcher_level = DiagStatus::OK;
      switcher_msg = "Switcher stable: " + sw.annotation;
      stamp_transitional_start_ = std::nullopt;
    } else {
      // 過渡状態 (INITIALIZING / WAIT_FOR_AUTOWARE / IN_ELECTION)
      if (!stamp_transitional_start_.has_value()) {
        stamp_transitional_start_ = now;
      }
      const double elapsed_ms = (now - *stamp_transitional_start_).seconds() * 1000.0;
      if (elapsed_ms > transitional_timeout_milli_) {
        switcher_level = DiagStatus::ERROR;
        switcher_msg = "Switcher transitional state too long (" +
                       std::to_string(static_cast<int>(elapsed_ms)) + "ms): " + sw.annotation;
      } else {
        switcher_level = DiagStatus::WARN;
        switcher_msg = "Switcher in transitional state: " + sw.annotation;
      }
    }
  }

  // ── AutowareReady ────────────────────────────────────────────────────
  auto autoware_level = DiagStatus::OK;
  std::string autoware_msg;
  if (!snap.autoware_ready.has_value()) {
    autoware_level = DiagStatus::WARN;
    autoware_msg = "Startup not yet complete: awaiting Autoware ready signal";
  } else {
    switch (snap.autoware_ready->value) {
      case AutowareReady::False:
        autoware_level = DiagStatus::WARN;
        autoware_msg = "Autoware is not ready";
        break;
      case AutowareReady::True:
        autoware_level = DiagStatus::OK;
        autoware_msg = "Autoware is ready";
        break;
    }
  }

  // ── VelocityStatus ───────────────────────────────────────────────────
  auto velocity_level = DiagStatus::OK;
  std::string velocity_msg;
  if (!snap.velocity_status.has_value()) {
    velocity_level = DiagStatus::WARN;
    velocity_msg = "Startup not yet complete: awaiting velocity data";
  } else {
    switch (snap.velocity_status->value) {
      case VelocityStatus::Stopped:
        velocity_level = DiagStatus::OK;
        velocity_msg = "Vehicle is stopped";
        break;
      case VelocityStatus::Moving:
        velocity_level = DiagStatus::OK;
        velocity_msg = "Vehicle is moving";
        break;
    }
  }

  // ── ControlMode ──────────────────────────────────────────────────────
  auto control_level = DiagStatus::OK;
  std::string control_msg;
  if (!snap.control_mode.has_value()) {
    control_level = DiagStatus::WARN;
    control_msg = "Startup not yet complete: awaiting control mode data";
  } else {
    switch (snap.control_mode->value) {
      case ControlMode::Manual:
        control_level = DiagStatus::OK;
        control_msg = "Manual control mode";
        break;
      case ControlMode::Auto:
        control_level = DiagStatus::OK;
        control_msg = "Autoware control mode";
        break;
    }
  }

  stat.add("switcher_signals", switcher_msg);
  stat.add("autoware_ready", autoware_msg);
  stat.add("velocity_status", velocity_msg);
  stat.add("control_mode", control_msg);

  // worst level で summary を設定
  const auto worst = std::max({switcher_level, autoware_level, velocity_level, control_level});
  if (worst == DiagStatus::OK) {
    stat.summary(DiagStatus::OK, "Redundancy switcher interface is running normally");
  } else if (worst == DiagStatus::WARN) {
    stat.summary(DiagStatus::WARN, "Redundancy switcher interface has warnings");
  } else if (worst == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, "Redundancy switcher interface has errors");
  } else {
    stat.summary(DiagStatus::STALE, "Redundancy switcher interface data is stale");
  }
}

}  // namespace redundancy_switcher

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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace redundancy_switcher
{

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

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

void SwitcherAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("SwitcherAdapter: node is null");
  if (!gateway) throw std::invalid_argument("SwitcherAdapter: gateway is null");
  node_ = node;
  gateway_ = gateway;

  const std::string sender_path = node_->declare_parameter<std::string>(
    "uds.switcher_command_path", "/tmp/redundancy_switcher_command.sock");
  const std::string receiver_path = node_->declare_parameter<std::string>(
    "uds.switcher_status_path", "/tmp/redundancy_switcher_status.sock");
  election_status_timeout_milli_ = node_->declare_parameter<double>(
    "uds.election_status_timeout_milli", 1000.0);
  is_main_ecu_ = node_->has_parameter("is_main_ecu")
    ? node_->get_parameter("is_main_ecu").as_bool()
    : node_->declare_parameter<bool>("is_main_ecu", true);

  uds_sender_ = std::make_unique<UdsSender<ElectionRequest>>(sender_path);
  uds_receiver_ = std::make_unique<UdsReceiver<ElectionStatus>>(
    receiver_path, /*use_nonblocking=*/false,
    [this](const ElectionStatus & s) { on_switcher_status(s); });

  const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(election_status_timeout_milli_ / 2.0));
  timeout_check_timer_ = node_->create_wall_timer(
    timeout_ns, [this]() { check_election_status_timeout(); });

  updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  updater_->setHardwareID("redundancy_switcher");
  updater_->add("main_ecu_fault", this, &SwitcherAdapter::update_main_ecu_fault_diag);
  updater_->add("sub_ecu_fault", this, &SwitcherAdapter::update_sub_ecu_fault_diag);
  updater_->add("main_vcu_fault", this, &SwitcherAdapter::update_main_vcu_fault_diag);
  updater_->add("sub_vcu_fault", this, &SwitcherAdapter::update_sub_vcu_fault_diag);
  updater_->add(
    "main_ecu_to_sub_ecu_link_fault", this,
    &SwitcherAdapter::update_main_ecu_to_sub_ecu_link_fault_diag);
  updater_->add(
    "main_ecu_to_main_vcu_link_fault", this,
    &SwitcherAdapter::update_main_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "main_ecu_to_sub_vcu_link_fault", this,
    &SwitcherAdapter::update_main_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_main_vcu_link_fault", this,
    &SwitcherAdapter::update_sub_ecu_to_main_vcu_link_fault_diag);
  updater_->add(
    "sub_ecu_to_sub_vcu_link_fault", this,
    &SwitcherAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag);
  updater_->add(
    "main_vcu_to_sub_vcu_link_fault", this,
    &SwitcherAdapter::update_main_vcu_to_sub_vcu_link_fault_diag);
}

// ---------------------------------------------------------------------------
// execute
// ---------------------------------------------------------------------------

void SwitcherAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const SelfInterruptionCommand &) {
        uds_sender_->send(ElectionRequest{/*.self_interruption=*/true, /*.reset=*/false});
      },
      [this](const ResetCommand &) {
        uds_sender_->send(ElectionRequest{/*.self_interruption=*/false, /*.reset=*/true});
      },
      [this](const UpdateAutowareReadyCommand & cmd) {
        autoware_ready_ = cmd.value;
      },
      [this](const UpdateAnotherEcuAvailabilityTimeoutCommand & cmd) {
        another_ecu_availability_timeout_ = cmd.timed_out;
      },
      [](const auto &) {}},
    command);
}

// ---------------------------------------------------------------------------
// on_switcher_status — UDS 受信コールバック
// ---------------------------------------------------------------------------

void SwitcherAdapter::on_switcher_status(const ElectionStatus & status)
{
  last_election_status_ = status;
  stamp_election_status_ = node_->now();
  check_switcher_connection();

  const auto annotation =
    node_state_to_string(status.node_state) + " leader=" + std::to_string(status.leader_id);
  gateway_->submit(
    InputEvent{SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{to_switcher_signals(status.node_state), annotation}}});
}

// ---------------------------------------------------------------------------
// check_election_status_timeout — 定期タイマーでタイムアウト検知
//
// on_switcher_status は受信時のみ呼ばれるため、Switcher が送信を止めると
// タイムアウトを検知できない。このタイマーが代わりに監視する。
// 次回タイマー起動時に再送しないようにする。
// ---------------------------------------------------------------------------

void SwitcherAdapter::check_election_status_timeout()
{
  if (!stamp_election_status_.has_value()) return;

  const auto now = node_->now();
  const double elapsed_ms = (now - *stamp_election_status_).seconds() * 1000.0;
  if (elapsed_ms > election_status_timeout_milli_) {
    gateway_->submit(InputEvent{
      SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{{false, false, true}, "Switcher data timeout"}}});
  }
}

// ---------------------------------------------------------------------------
// node_state_to_string — 人間可読な状態名に変換
// ---------------------------------------------------------------------------

std::string SwitcherAdapter::node_state_to_string(uint8_t node_state)
{
  switch (node_state) {
    case 0: return "INITIALIZING";
    case 1: return "ELECTABLE";
    case 2: return "WAIT_FOR_AUTOWARE";
    case 3: return "IN_ELECTION";
    case 4: return "ELECTION_COMPLETED";
    case 5: return "ELECTION_UNCLOSED";
    case 6: return "PATH_NOT_FOUND";
    case 7: return "SELF_INTERRUPTION";
    default: return "UNKNOWN(" + std::to_string(node_state) + ")";
  }
}

// ---------------------------------------------------------------------------
// to_switcher_signals — ElectionStatus.node_state → SwitcherSignals 変換
//
// 各 node_state が3信号のどれに対応するかは互いに排他的で一意に決まる。
//   is_stable:           ELECTABLE(1) / ELECTION_COMPLETED(4)
//   is_self_interrupted: SELF_INTERRUPTION(7)
//   is_faulted:          ELECTION_UNCLOSED(5) / PATH_NOT_FOUND(6)
//   全て false:          INITIALIZING(0) / WAIT_FOR_AUTOWARE(2) / IN_ELECTION(3) / 未知値
// ---------------------------------------------------------------------------

SwitcherSignals SwitcherAdapter::to_switcher_signals(uint8_t node_state)
{
  switch (node_state) {
    case 1:  // ELECTABLE
    case 4:  // ELECTION_COMPLETED
      return {true, false, false};
    case 7:  // SELF_INTERRUPTION
      return {false, true, false};
    case 5:  // ELECTION_UNCLOSED
    case 6:  // PATH_NOT_FOUND
      return {false, false, true};
    default:  // INITIALIZING / WAIT_FOR_AUTOWARE / IN_ELECTION / 未知値
      return {false, false, false};
  }
}

void SwitcherAdapter::check_switcher_connection()
{
  node_fault_points_.clear();
  link_fault_points_.clear();

  if (!last_election_status_.has_value()) return;
  const auto & s = *last_election_status_;

  if (s.node_state == 0 /* INITIALIZING */ || s.node_state == 3 /* IN_ELECTION */) return;

  if (!autoware_ready_ || *autoware_ready_ != AutowareReady::True) return;

  // judge: 全接続を「信頼済み」で初期化し、自 ECU から到達できないノードの発信を false 化する
  ElectionStatus judge = s;
  judge.main_ecu_to_main_ecu_connected = judge.main_ecu_to_sub_ecu_connected =
  judge.main_ecu_to_main_vcu_connected = judge.main_ecu_to_sub_vcu_connected =
  judge.sub_ecu_to_main_ecu_connected  = judge.sub_ecu_to_sub_ecu_connected  =
  judge.sub_ecu_to_main_vcu_connected  = judge.sub_ecu_to_sub_vcu_connected  =
  judge.main_vcu_to_main_ecu_connected = judge.main_vcu_to_sub_ecu_connected =
  judge.main_vcu_to_main_vcu_connected = judge.main_vcu_to_sub_vcu_connected =
  judge.sub_vcu_to_main_ecu_connected  = judge.sub_vcu_to_sub_ecu_connected  =
  judge.sub_vcu_to_main_vcu_connected  = judge.sub_vcu_to_sub_vcu_connected  = true;

  // 信頼伝播: 自 ECU からリモートへの接続が切れている場合、そのノードの全発信を信頼しない
  auto distrust = [](bool connected, bool & a, bool & b, bool & c, bool & d) {
    if (!connected) a = b = c = d = false;
  };

  if (is_main_ecu_) {
    if (!s.main_ecu_to_sub_ecu_connected && !s.main_ecu_to_main_vcu_connected &&
        !s.main_ecu_to_sub_vcu_connected) {
      node_fault_points_.insert("main_ecu");
      return;
    }
    distrust(s.main_ecu_to_sub_ecu_connected,
      judge.sub_ecu_to_main_ecu_connected, judge.sub_ecu_to_sub_ecu_connected,
      judge.sub_ecu_to_main_vcu_connected, judge.sub_ecu_to_sub_vcu_connected);
    distrust(s.main_ecu_to_main_vcu_connected,
      judge.main_vcu_to_main_ecu_connected, judge.main_vcu_to_sub_ecu_connected,
      judge.main_vcu_to_main_vcu_connected, judge.main_vcu_to_sub_vcu_connected);
    distrust(s.main_ecu_to_sub_vcu_connected,
      judge.sub_vcu_to_main_ecu_connected, judge.sub_vcu_to_sub_ecu_connected,
      judge.sub_vcu_to_main_vcu_connected, judge.sub_vcu_to_sub_vcu_connected);
  } else {
    if (!s.sub_ecu_to_main_ecu_connected && !s.sub_ecu_to_main_vcu_connected &&
        !s.sub_ecu_to_sub_vcu_connected) {
      node_fault_points_.insert("sub_ecu");
      return;
    }
    distrust(s.sub_ecu_to_main_ecu_connected,
      judge.main_ecu_to_main_ecu_connected, judge.main_ecu_to_sub_ecu_connected,
      judge.main_ecu_to_main_vcu_connected, judge.main_ecu_to_sub_vcu_connected);
    distrust(s.sub_ecu_to_main_vcu_connected,
      judge.main_vcu_to_main_ecu_connected, judge.main_vcu_to_sub_ecu_connected,
      judge.main_vcu_to_main_vcu_connected, judge.main_vcu_to_sub_vcu_connected);
    distrust(s.sub_ecu_to_sub_vcu_connected,
      judge.sub_vcu_to_main_ecu_connected, judge.sub_vcu_to_sub_ecu_connected,
      judge.sub_vcu_to_main_vcu_connected, judge.sub_vcu_to_sub_vcu_connected);
  }

  // ノード故障検出: 全着信方向が「信頼不可 OR 実際に切断」→ 故障
  auto unreachable = [](bool jf, bool af) { return !jf || !af; };

  if (unreachable(judge.sub_ecu_to_main_ecu_connected,  s.sub_ecu_to_main_ecu_connected) &&
      unreachable(judge.main_vcu_to_main_ecu_connected, s.main_vcu_to_main_ecu_connected) &&
      unreachable(judge.sub_vcu_to_main_ecu_connected,  s.sub_vcu_to_main_ecu_connected))
    node_fault_points_.insert("main_ecu");

  if (unreachable(judge.main_ecu_to_sub_ecu_connected, s.main_ecu_to_sub_ecu_connected) &&
      unreachable(judge.main_vcu_to_sub_ecu_connected, s.main_vcu_to_sub_ecu_connected) &&
      unreachable(judge.sub_vcu_to_sub_ecu_connected,  s.sub_vcu_to_sub_ecu_connected))
    node_fault_points_.insert("sub_ecu");

  if (unreachable(judge.main_ecu_to_main_vcu_connected, s.main_ecu_to_main_vcu_connected) &&
      unreachable(judge.sub_ecu_to_main_vcu_connected,  s.sub_ecu_to_main_vcu_connected) &&
      unreachable(judge.sub_vcu_to_main_vcu_connected,  s.sub_vcu_to_main_vcu_connected))
    node_fault_points_.insert("main_vcu");

  if (unreachable(judge.main_ecu_to_sub_vcu_connected, s.main_ecu_to_sub_vcu_connected) &&
      unreachable(judge.sub_ecu_to_sub_vcu_connected,  s.sub_ecu_to_sub_vcu_connected) &&
      unreachable(judge.main_vcu_to_sub_vcu_connected, s.main_vcu_to_sub_vcu_connected))
    node_fault_points_.insert("sub_vcu");

  // リンク故障検出: judge=true かつ両端正常なのに actual=false → リンク故障
  const bool mef = node_fault_points_.count("main_ecu");
  const bool sef = node_fault_points_.count("sub_ecu");
  const bool mvf = node_fault_points_.count("main_vcu");
  const bool svf = node_fault_points_.count("sub_vcu");

  auto mark_link = [&](bool jf, bool af, bool src_f, bool dst_f, const std::string & key) {
    if (jf && !src_f && !dst_f && !af) link_fault_points_.insert(key);
  };

  // clang-format off
  mark_link(judge.main_ecu_to_sub_ecu_connected,  s.main_ecu_to_sub_ecu_connected,  mef, sef, "main_ecu_to_sub_ecu_link");
  mark_link(judge.sub_ecu_to_main_ecu_connected,  s.sub_ecu_to_main_ecu_connected,  sef, mef, "main_ecu_to_sub_ecu_link");
  mark_link(judge.main_ecu_to_main_vcu_connected, s.main_ecu_to_main_vcu_connected, mef, mvf, "main_ecu_to_main_vcu_link");
  mark_link(judge.main_vcu_to_main_ecu_connected, s.main_vcu_to_main_ecu_connected, mvf, mef, "main_ecu_to_main_vcu_link");
  mark_link(judge.main_ecu_to_sub_vcu_connected,  s.main_ecu_to_sub_vcu_connected,  mef, svf, "main_ecu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_main_ecu_connected,  s.sub_vcu_to_main_ecu_connected,  svf, mef, "main_ecu_to_sub_vcu_link");
  mark_link(judge.sub_ecu_to_main_vcu_connected,  s.sub_ecu_to_main_vcu_connected,  sef, mvf, "sub_ecu_to_main_vcu_link");
  mark_link(judge.main_vcu_to_sub_ecu_connected,  s.main_vcu_to_sub_ecu_connected,  mvf, sef, "sub_ecu_to_main_vcu_link");
  mark_link(judge.sub_ecu_to_sub_vcu_connected,   s.sub_ecu_to_sub_vcu_connected,   sef, svf, "sub_ecu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_sub_ecu_connected,   s.sub_vcu_to_sub_ecu_connected,   svf, sef, "sub_ecu_to_sub_vcu_link");
  mark_link(judge.main_vcu_to_sub_vcu_connected,  s.main_vcu_to_sub_vcu_connected,  mvf, svf, "main_vcu_to_sub_vcu_link");
  mark_link(judge.sub_vcu_to_main_vcu_connected,  s.sub_vcu_to_main_vcu_connected,  svf, mvf, "main_vcu_to_sub_vcu_link");
  // clang-format on
}

// ---------------------------------------------------------------------------
// ノード/リンク故障 診断コールバック群 — check_switcher_connection() の結果を publish
// ---------------------------------------------------------------------------

bool SwitcherAdapter::no_data(diagnostic_updater::DiagnosticStatusWrapper & stat) const
{
  if (last_election_status_.has_value()) return false;
  stat.summary(DiagStatus::STALE, "No data received from switcher");
  return true;
}

void SwitcherAdapter::update_main_ecu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  if (last_election_status_->node_state == 7 /* SELF_INTERRUPTION */) {
    stat.summary(DiagStatus::OK, "Main ECU is healthy");
    return;
  }
  const bool fault = node_fault_points_.count("main_ecu");
  // main ecu
  if (is_main_ecu_ && fault) {
    stat.summary(DiagStatus::ERROR, "Main ECU fault detected");
    return;
  }
  // sub ecu
  if (!is_main_ecu_ && another_ecu_availability_timeout_ && fault) {
    stat.summary(DiagStatus::ERROR, "Main ECU fault detected");
    return;
  }
  stat.summary(DiagStatus::OK, "Main ECU is healthy");
}

void SwitcherAdapter::update_sub_ecu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const bool fault = node_fault_points_.count("sub_ecu");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU fault detected" : "Sub ECU is healthy");
}

void SwitcherAdapter::update_main_vcu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const bool fault = node_fault_points_.count("main_vcu");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main VCU fault detected" : "Main VCU is healthy");
}

void SwitcherAdapter::update_sub_vcu_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const bool fault = node_fault_points_.count("sub_vcu");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub VCU fault detected" : "Sub VCU is healthy");
}

void SwitcherAdapter::update_main_ecu_to_sub_ecu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("main_ecu_to_sub_ecu", s.main_ecu_to_sub_ecu_connected);
  stat.add("sub_ecu_to_main_ecu", s.sub_ecu_to_main_ecu_connected);
  const bool fault = link_fault_points_.count("main_ecu_to_sub_ecu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main-Sub ECU link fault detected" : "Main-Sub ECU link is healthy");
}

void SwitcherAdapter::update_main_ecu_to_main_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("main_ecu_to_main_vcu", s.main_ecu_to_main_vcu_connected);
  stat.add("main_vcu_to_main_ecu", s.main_vcu_to_main_ecu_connected);
  const bool fault = link_fault_points_.count("main_ecu_to_main_vcu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main ECU to Main VCU link fault detected" : "Main ECU to Main VCU link is healthy");
}

void SwitcherAdapter::update_main_ecu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("main_ecu_to_sub_vcu", s.main_ecu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_main_ecu", s.sub_vcu_to_main_ecu_connected);
  const bool fault = link_fault_points_.count("main_ecu_to_sub_vcu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main ECU to Sub VCU link fault detected" : "Main ECU to Sub VCU link is healthy");
}

void SwitcherAdapter::update_sub_ecu_to_main_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("sub_ecu_to_main_vcu", s.sub_ecu_to_main_vcu_connected);
  stat.add("main_vcu_to_sub_ecu", s.main_vcu_to_sub_ecu_connected);
  const bool fault = link_fault_points_.count("sub_ecu_to_main_vcu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU to Main VCU link fault detected" : "Sub ECU to Main VCU link is healthy");
}

void SwitcherAdapter::update_sub_ecu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("sub_ecu_to_sub_vcu", s.sub_ecu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_sub_ecu", s.sub_vcu_to_sub_ecu_connected);
  const bool fault = link_fault_points_.count("sub_ecu_to_sub_vcu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Sub ECU to Sub VCU link fault detected" : "Sub ECU to Sub VCU link is healthy");
}

void SwitcherAdapter::update_main_vcu_to_sub_vcu_link_fault_diag(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (no_data(stat)) return;
  const auto & s = *last_election_status_;
  stat.add("main_vcu_to_sub_vcu", s.main_vcu_to_sub_vcu_connected);
  stat.add("sub_vcu_to_main_vcu", s.sub_vcu_to_main_vcu_connected);
  const bool fault = link_fault_points_.count("main_vcu_to_sub_vcu_link");
  stat.summary(fault ? DiagStatus::ERROR : DiagStatus::OK,
               fault ? "Main-Sub VCU link fault detected" : "Main-Sub VCU link is healthy");
}

}  // namespace redundancy_switcher

PLUGINLIB_EXPORT_CLASS(redundancy_switcher::SwitcherAdapter, redundancy_switcher::IAdapterPlugin)

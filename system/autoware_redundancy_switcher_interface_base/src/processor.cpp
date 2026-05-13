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
#include "redundancy_switcher_interface_base/core_logic/processor.hpp"

#include <string>
#include <variant>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// overloaded helper (C++17) — std::visit で複数の callable を束ねる
// ---------------------------------------------------------------------------

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// ---------------------------------------------------------------------------
// handle — 唯一の公開エントリポイント
// ---------------------------------------------------------------------------

Processor::Processor()
{
  state_.interface_diags.add_status(
    "autoware_ready", DiagLevelIR::Stale, "Not received any status yet");
  state_.interface_diags.add_status(
    "velocity_status", DiagLevelIR::Stale, "Not received any status yet");
  state_.interface_diags.add_status(
    "control_mode", DiagLevelIR::Stale, "Not received any status yet");
  state_.switcher_diags.add_status(
    "switcher_status", DiagLevelIR::Stale, "Not received any status yet");
}

std::vector<OutputCommand> Processor::handle(const InputEvent & event)
{
  return std::visit(
    overloaded{
      [this](const SelfInterruptionEvent & e) { return self_interruption(e); },
      [this](const ResetEvent & e) { return reset(e); },
      [this](const SetAutowareReadyEvent & e) { return set_autoware_ready(e); },
      [this](const SetVehicleStoppedEvent & e) { return set_vehicle_stopped(e); },
      [this](const SetAutowareControlEvent & e) { return set_autoware_control(e); },
      [this](const SwitcherDataTimeoutEvent & e) { return switcher_data_timeout(e); }},
    event);
}

std::vector<OutputCommand> Processor::self_interruption(const SelfInterruptionEvent &)
{
  if (state_.autoware_ready != AutowareReady::True) {
    return {LogCommand{LogLevel::Debug, "Self-interruption rejected: Autoware is not ready."}};
  }
  if (state_.control_mode != ControlMode::Auto) {
    return {LogCommand{LogLevel::Debug, "Self-interruption rejected: Autoware is not in control."}};
  }

  const auto & status = state_.switcher_status;
  switch (status.value) {
    case SwitcherStatus::Unknown:
      return {
        LogCommand{LogLevel::Error, "Self-interruption rejected: switcher status is unknown."}};
    case SwitcherStatus::Pending:
      return {LogCommand{LogLevel::Info, "Self-interruption rejected: " + status.annotation}};
    case SwitcherStatus::Stable:
      return {
        SelfInterruptionCommand{},
        LogCommand{LogLevel::Info, "Self-interruption accepted: " + status.annotation}};
    case SwitcherStatus::Degraded:
      return {
        SelfInterruptionCommand{},
        LogCommand{LogLevel::Warn, "Self-interruption accepted: " + status.annotation}};
    case SwitcherStatus::Impaired:
      return {LogCommand{LogLevel::Error, "Self-interruption rejected: " + status.annotation}};
    default:
      return {LogCommand{LogLevel::Error, "Self-interruption rejected: invalid switcher status."}};
  }
}

std::vector<OutputCommand> Processor::reset(const ResetEvent &)
{
  if (state_.velocity_status != VelocityStatus::Stopped) {
    return {LogCommand{LogLevel::Warn, "Reset rejected: vehicle is not stopped."}};
  }

  const auto & status = state_.switcher_status;

  // Autoware 未準備時はリセットを常に受け入れる
  if (state_.autoware_ready != AutowareReady::True) {
    return {ResetCommand{}, LogCommand{LogLevel::Info, "Reset accepted: " + status.annotation}};
  }

  switch (status.value) {
    case SwitcherStatus::Unknown:
      return {LogCommand{LogLevel::Error, "Reset rejected: switcher status is unknown."}};
    case SwitcherStatus::Pending:
      return {LogCommand{LogLevel::Info, "Reset rejected: " + status.annotation}};
    case SwitcherStatus::Stable:
      return {LogCommand{LogLevel::Info, "Reset ignored: " + status.annotation}};
    case SwitcherStatus::Degraded:
      return {ResetCommand{}, LogCommand{LogLevel::Info, "Reset accepted: " + status.annotation}};
    case SwitcherStatus::Impaired:
      return {ResetCommand{}, LogCommand{LogLevel::Info, "Reset accepted: " + status.annotation}};
    default:
      return {LogCommand{LogLevel::Error, "Reset rejected: invalid switcher status."}};
  }
}

std::vector<OutputCommand> Processor::switcher_data_timeout(const SwitcherDataTimeoutEvent &)
{
  // TODO(kawaguchi): Implement handling of switcher data timeout event
  return {
    LogCommand{LogLevel::Warn, "Switcher data timeout received. Handling not implemented yet."}};
}

std::vector<OutputCommand> Processor::set_autoware_ready(const SetAutowareReadyEvent & e)
{
  state_.autoware_ready = {e.is_ready ? AutowareReady::True : AutowareReady::False, ""};
  return {
    LogCommand{LogLevel::Info, std::string("Autoware ready: ") + (e.is_ready ? "true" : "false")}};
}

std::vector<OutputCommand> Processor::set_vehicle_stopped(const SetVehicleStoppedEvent & e)
{
  state_.velocity_status = {e.is_stopped ? VelocityStatus::Stopped : VelocityStatus::Moving, ""};
  return {LogCommand{
    LogLevel::Info, std::string("Vehicle stopped: ") + (e.is_stopped ? "true" : "false")}};
}

std::vector<OutputCommand> Processor::set_autoware_control(const SetAutowareControlEvent & e)
{
  state_.control_mode = {e.is_autoware_control ? ControlMode::Auto : ControlMode::Manual, ""};
  return {LogCommand{
    LogLevel::Info,
    std::string("Autoware control: ") + (e.is_autoware_control ? "true" : "false")}};
}

}  // namespace redundancy_switcher

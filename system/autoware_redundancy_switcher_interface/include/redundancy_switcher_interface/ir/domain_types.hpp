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

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// SwitcherSignals — Switcher から受け取る選出状態を3つの直交する信号で表現する
//
// node_state の8値を意味ごとに分解したもの。3値は互いに排他的。
//   is_stable:           ELECTABLE / ELECTION_COMPLETED
//   is_self_interrupted: SELF_INTERRUPTION
//   is_faulted:          ELECTION_UNCLOSED / PATH_NOT_FOUND
//   全て false:          過渡状態 (INITIALIZING / WAIT_FOR_AUTOWARE / IN_ELECTION)
//
// Processor はこの信号のみを参照し、node_state の具体値を知らない。
// ---------------------------------------------------------------------------

struct SwitcherSignals {
  bool is_stable;           // 選出完了・自己割り込み可能
  bool is_self_interrupted; // 自己割り込み承認済み（reset のみ受け付け）
  bool is_faulted;          // 選出破綻（どちらの操作も拒否、Error ログ）
};

enum class AutowareReady {
  False,  // Not ready for switching function
  True,   // Ready for switching function
};

enum class VelocityStatus {
  Stopped,  // Vehicle is stopped
  Moving,   // Vehicle is moving
};

enum class ControlMode {
  Manual,  // Manual control mode
  Auto,    // Autonomous control mode
};

// ---------------------------------------------------------------------------
// Annotated<E> — enum 値に補足情報（注釈文字列）を付与するラッパー
//
// enum 値そのものの意味は変えず、デバッグや診断のための補足情報を添付する。
//
// 使い方:
//   state_.switcher_status = Annotated{SwitcherStatus::Stable, "Operator requested stable mode"};
//   if (state_.switcher_status->value == SwitcherStatus::Stable) { ... }
//   RCLCPP_DEBUG(logger_, "switcher_status: %s", state_.switcher_status->annotation.c_str());
// ---------------------------------------------------------------------------

template <typename E>
struct Annotated
{
  E value;
  std::string annotation;

  Annotated(E v, std::string r = {}) : value(v), annotation(std::move(r)) {}  // NOLINT
};

// ---------------------------------------------------------------------------
// Aliases
// ---------------------------------------------------------------------------

using RequestId = uint64_t;
using TimerId = std::string;

// ---------------------------------------------------------------------------
// Snapshot
// Read-only view of Processor's current state. Returned by snapshot() and
// embedded in output effects so Adapters can serialize/publish without
// knowing Processor internals.
//
// 各フィールドが nullopt = そのデータを一度も受信していない（起動準備未完了）
// ---------------------------------------------------------------------------

struct DomainSnapshot
{
  std::optional<Annotated<AutowareReady>> autoware_ready;
  std::optional<Annotated<VelocityStatus>> velocity_status;
  std::optional<Annotated<ControlMode>> control_mode;

  // nullopt = 未受信 / タイムアウト後リセット
  // .value      = SwitcherSignals（3つの直交信号）
  // .annotation = 人間可読な状態説明 ("SELF_INTERRUPTION leader=MAIN_ECU" 等)
  std::optional<Annotated<SwitcherSignals>> switcher;

  // 相手 ECU からの CommandModeAvailability が一定時間届かない場合に true
  // nullopt = 未評価（起動直後）
  std::optional<bool> another_ecu_availability_timeout;
};

}  // namespace redundancy_switcher

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

#include "redundancy_switcher_interface/ir/domain_types.hpp"

#include <string>
#include <variant>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// InputEvent — "何が起きたか" を表す型
//
// Processor はこれらのイベントを受け取って状態遷移・エフェクト決定を行う。
// Adapter はこれらを生成してからのみ Processor.handle() を呼ぶ。
// Processor はイベント発生源（ROS, UDS, etc.）を一切知らない。
//
// 各イベントは Annotated<T> で包まれており、T が具体的なデータ、
// annotation が IR 変換時に必要な実装固有の補足情報を保持する。
// ---------------------------------------------------------------------------

// Autowareからのイベント
/// 自 ECU がエラーを検出し、上位系への通知が必要な場合
struct SelfInterruptionEvent
{
  Annotated<std::monostate> value{{}, ""};
};

/// オペレータ等からの初期化リセット要求
struct ResetEvent
{
  Annotated<std::monostate> value{{}, ""};
};

struct SetAutowareReadyEvent
{
  Annotated<AutowareReady> value;
};

struct SetVelocityStatusEvent
{
  Annotated<VelocityStatus> value;
};

struct SetControlModeEvent
{
  Annotated<ControlMode> value;
};

// Autowareからのイベント (サブシステム)
/// 相手 ECU からの CommandModeAvailability が一定時間届かない場合のタイムアウト通知
struct SetAnotherEcuAvailabilityTimeoutEvent
{
  Annotated<bool> timed_out;
};

// Switcherからのイベント
/// Switcher から選出状態が更新された
/// タイムアウト・接続断は is_faulted=true の SwitcherSignals として表現する
struct SetSwitcherSignalsEvent
{
  Annotated<SwitcherSignals> value;
};

struct SetActiveControlUnitEvent
{
  Annotated<std::vector<uint8_t>> unit_ids;
};

// ---------------------------------------------------------------------------
// InputEvent variant — Processor の唯一の入口型
// ---------------------------------------------------------------------------

using InputEvent = std::variant<
  SelfInterruptionEvent, ResetEvent, SetAutowareReadyEvent, SetVelocityStatusEvent,
  SetControlModeEvent, SetSwitcherSignalsEvent, SetActiveControlUnitEvent,
  SetAnotherEcuAvailabilityTimeoutEvent>;

}  // namespace redundancy_switcher

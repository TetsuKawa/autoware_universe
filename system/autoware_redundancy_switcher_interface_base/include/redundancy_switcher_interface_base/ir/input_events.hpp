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

#include "redundancy_switcher_interface_base/ir/domain_types.hpp"

#include <string>
#include <variant>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// InputEvent — "何が起きたか" を表す型
//
// Processor はこれらのイベントを受け取って状態遷移・エフェクト決定を行う。
// Adapter はこれらを生成してからのみ Processor.handle() を呼ぶ。
// Processor はイベント発生源（ROS, UDS, etc.）を一切知らない。
// ---------------------------------------------------------------------------

// Autowareからのイベント
/// 自 ECU がエラーを検出し、上位系への通知が必要な場合
struct SelfInterruptionEvent
{
};

/// オペレータ等からの初期化リセット要求
struct ResetEvent
{
};

struct SetAutowareReadyEvent
{
  bool is_ready;
};

struct SetVehicleStoppedEvent
{
  bool is_stopped;
};

struct SetAutowareControlEvent
{
  bool is_autoware_control;
};

// Switcherからのイベント
struct SwitcherDataTimeoutEvent
{
};

// ---------------------------------------------------------------------------
// InputEvent variant — Processor の唯一の入口型
// ---------------------------------------------------------------------------

using InputEvent = std::variant<
  SelfInterruptionEvent, ResetEvent, SetAutowareReadyEvent, SetVehicleStoppedEvent,
  SetAutowareControlEvent, SwitcherDataTimeoutEvent>;

}  // namespace redundancy_switcher

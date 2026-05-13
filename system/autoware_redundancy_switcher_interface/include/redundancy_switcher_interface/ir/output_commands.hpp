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

struct ResetCommand
{
};

struct SelfInterruptionCommand
{
};

// ── ロギング (LogAdapter が処理) ──────────────────────────────────────────
enum class LogLevel { Debug, Info, Warn, Error, Fatal };

struct LogCommand
{
  LogLevel level{LogLevel::Info};
  std::string message;
};

/// Processor の状態変化を通知する。DiagAdapter が受け取り診断を更新する。
/// snapshot フィールドに診断に必要な状態が埋め込まれているため、
/// DiagAdapter は EventGateway への逆参照（snapshot() 呼び出し）が不要。
struct UpdateStatusDiagCommand
{
  DomainSnapshot snapshot;
};

struct UpdateActiveControlUnitCommand
{
  std::vector<uint8_t> unit_ids;
};

/// reset() の受理/拒否を呼び出し元（SubSystemAdapter）に伝える結果コマンド。
/// CommandBus を通じて全 Adapter に配信されるが、SubSystemAdapter 以外は無視する。
///
/// reason の使われ方（SubSystemAdapter が ResponseStatus にマップする）:
///   accepted=true                     → SUCCESS
///   accepted=false, Ignored           → IGNORED  (条件未達: 走行中など)
///   accepted=false, NotNecessary      → SUCCESS   (既に安定状態: reset 不要)
///   accepted=false, Error             → ERROR     (故障・異常状態)
enum class ResetRejectedReason { Ignored, NotNecessary, Error };

struct ResetResultCommand
{
  bool accepted;
  ResetRejectedReason reason{ResetRejectedReason::Error};  // accepted=false のときのみ有効
  std::string message;
};

/// SwitcherAdapter が autoware_ready をローカルキャッシュするための通知コマンド。
/// SetAutowareReadyEvent が処理されるたびに Processor が emit する。
struct UpdateAutowareReadyCommand
{
  AutowareReady value;
};

/// SwitcherAdapter が相手 ECU availability タイムアウト状態をキャッシュするための通知コマンド。
/// SetAnotherEcuAvailabilityTimeoutEvent が処理されるたびに Processor が emit する。
struct UpdateAnotherEcuAvailabilityTimeoutCommand
{
  bool timed_out;
};

// ---------------------------------------------------------------------------
// OutputCommand variant — Processor の唯一の出力型
// ---------------------------------------------------------------------------

using OutputCommand = std::variant<
  LogCommand, ResetCommand, SelfInterruptionCommand, UpdateStatusDiagCommand,
  UpdateActiveControlUnitCommand, UpdateAutowareReadyCommand, ResetResultCommand,
  UpdateAnotherEcuAvailabilityTimeoutCommand>;

}  // namespace redundancy_switcher

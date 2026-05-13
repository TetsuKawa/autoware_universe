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

struct UpdateIntefaceDiagCommand
{
};

struct UpdateSwitcherDiagCommand
{
};

struct UpdateNotificationDiagCommand
{
};

struct UpdateActiveControlUnitCommand
{
  std::vector<uint8_t> unit_ids;
};

// ---------------------------------------------------------------------------
// OutputCommand variant — Processor の唯一の出力型
// ---------------------------------------------------------------------------

using OutputCommand = std::variant<
  LogCommand, ResetCommand, SelfInterruptionCommand, UpdateIntefaceDiagCommand,
  UpdateSwitcherDiagCommand, UpdateNotificationDiagCommand, UpdateActiveControlUnitCommand>;

}  // namespace redundancy_switcher

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
#include <map>
#include <string>
#include <utility>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

enum class SwitcherStatus {
  Unknown,   // Not received any status yet
  Pending,   // Not ready for swithing function. Reject both self-interruption and reset requests.
  Stable,    // Ready for switching function. Accept self-interruption requests, but reject reset
             // requests (no need to reset if already stable).
  Degraded,  // Partially not ready for switching function due to some degradation. Accept both
             // self-interruption and reset requests.
  Impaired,  // Not ready for switching function due to some impairment. Accept reset requests, but
             // reject self-interruption requests.
};

enum class AutowareReady {
  Unknown,  // Not received any status yet
  False,    // Not ready for switching function
  True,     // Ready for switching function
};

enum class VelocityStatus {
  Unknown,  // Not received any status yet
  Stopped,  // Vehicle is stopped
  Moving,   // Vehicle is moving
};

enum class ControlMode {
  Unknown,  // Not received any status yet
  Manual,   // Manual control mode
  Auto,     // Autonomous control mode
};

enum class DiagLevelIR {
  Ok = 0,
  Warn = 1,
  Error = 2,
  Stale = 3,
};

// clang-format off
inline bool operator<(DiagLevelIR a, DiagLevelIR b) noexcept
{
  return static_cast<int>(a) < static_cast<int>(b);
}
inline bool operator>(DiagLevelIR a, DiagLevelIR b) noexcept { return b < a; }
inline bool operator<=(DiagLevelIR a, DiagLevelIR b) noexcept { return !(a > b); }
inline bool operator>=(DiagLevelIR a, DiagLevelIR b) noexcept { return !(a < b); }
// clang-format on

class DiagValueIR
{
public:
  DiagLevelIR level{DiagLevelIR::Stale};
  std::string message;

  std::string level_string() const
  {
    switch (level) {
      case DiagLevelIR::Ok:
        return "OK";
      case DiagLevelIR::Warn:
        return "WARN";
      case DiagLevelIR::Error:
        return "ERROR";
      case DiagLevelIR::Stale:
        return "STALE";
      default:
        return "UNKNOWN";
    }
  }
};

class DiagKeyValueIR
{
public:
  DiagKeyValueIR() = default;
  std::map<std::string, DiagValueIR> values;

  void add_status(const std::string & key, DiagLevelIR level, const std::string & message)
  {
    values[key] = DiagValueIR{level, message};
  }

  DiagKeyValueIR operator+(const DiagKeyValueIR & other) const
  {
    DiagKeyValueIR result = *this;
    for (const auto & [key, value] : other.values) {
      // If the same key exists, keep the one with worse level
      if (result.values.find(key) != result.values.end()) {
        if (value.level > result.values[key].level) {
          result.values[key] = value;
        }
      } else {
        result.values[key] = value;
      }
    }
    return result;
  }
};

// ---------------------------------------------------------------------------
// Annotated<E> — enum 値に補足情報（注釈文字列）を付与するラッパー
//
// enum 値そのものの意味は変えず、デバッグや診断のための補足情報を添付する。
//
// 使い方:
//   state_.switcher_status = {SwitcherStatus::Stable, "Operator requested stable mode"};
//   if (state_.switcher_status == SwitcherStatus::Stable) { ... }  // enum との比較はそのまま
//   RCLCPP_DEBUG(logger_, "switcher_status: %s", state_.switcher_status.annotation.c_str());
// ---------------------------------------------------------------------------

template <typename E>
struct Annotated
{
  E value{};
  std::string annotation;

  Annotated() = default;
  Annotated(E v, std::string r = {}) : value(v), annotation(std::move(r)) {}  // NOLINT

  bool operator==(E other) const { return value == other; }
  bool operator!=(E other) const { return value != other; }
  bool operator==(const Annotated & o) const { return value == o.value; }
  bool operator!=(const Annotated & o) const { return value != o.value; }
};

// enum == Annotated<E> の順番でも比較できるようにする
template <typename E>
bool operator==(E lhs, const Annotated<E> & rhs)
{
  return lhs == rhs.value;
}
template <typename E>
bool operator!=(E lhs, const Annotated<E> & rhs)
{
  return lhs != rhs.value;
}

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
// ---------------------------------------------------------------------------

struct DomainSnapshot
{
  Annotated<SwitcherStatus> switcher_status{SwitcherStatus::Unknown};
  Annotated<AutowareReady> autoware_ready{AutowareReady::Unknown};
  Annotated<VelocityStatus> velocity_status{VelocityStatus::Unknown};
  Annotated<ControlMode> control_mode{ControlMode::Unknown};
  DiagKeyValueIR interface_diags;
  DiagKeyValueIR switcher_diags;
  DiagKeyValueIR notification_diags;
};

}  // namespace redundancy_switcher

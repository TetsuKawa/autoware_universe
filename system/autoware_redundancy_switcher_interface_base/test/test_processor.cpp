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

#include <gtest/gtest.h>

#include <vector>

namespace redundancy_switcher
{

template <typename T>
const T * find_effect(const std::vector<OutputCommand> & commands)
{
  for (const auto & e : commands) {
    if (const auto * p = std::get_if<T>(&e)) return p;
  }
  return nullptr;
}

template <typename T>
bool has_effect(const std::vector<OutputCommand> & commands)
{
  return find_effect<T>(commands) != nullptr;
}

class ProcessorTest : public ::testing::Test
{
protected:
  Processor processor_;

  void set_ready() { processor_.handle(InputEvent{SetAutowareReadyEvent{true}}); }
  void set_stopped() { processor_.handle(InputEvent{SetVehicleStoppedEvent{true}}); }
  void set_auto_control() { processor_.handle(InputEvent{SetAutowareControlEvent{true}}); }
};

// ── 初期状態 ──────────────────────────────────────────────────────────────

TEST_F(ProcessorTest, InitialState_IsUnknown)
{
  const auto snap = processor_.snapshot();
  EXPECT_EQ(snap.switcher_status, SwitcherStatus::Unknown);
  EXPECT_EQ(snap.autoware_ready, AutowareReady::Unknown);
  EXPECT_EQ(snap.velocity_status, VelocityStatus::Unknown);
  EXPECT_EQ(snap.control_mode, ControlMode::Unknown);
}

// ── SetAutowareReady ────────────────────────────────────────────────────────

TEST_F(ProcessorTest, SetAutowareReady_UpdatesState)
{
  const auto commands = processor_.handle(InputEvent{SetAutowareReadyEvent{true}});
  EXPECT_EQ(processor_.snapshot().autoware_ready, AutowareReady::True);
  EXPECT_TRUE(has_effect<LogCommand>(commands));
}

TEST_F(ProcessorTest, SetAutowareReady_False_UpdatesState)
{
  processor_.handle(InputEvent{SetAutowareReadyEvent{true}});
  processor_.handle(InputEvent{SetAutowareReadyEvent{false}});
  EXPECT_EQ(processor_.snapshot().autoware_ready, AutowareReady::False);
}

// ── SetVehicleStopped ───────────────────────────────────────────────────────

TEST_F(ProcessorTest, SetVehicleStopped_UpdatesState)
{
  processor_.handle(InputEvent{SetVehicleStoppedEvent{true}});
  EXPECT_EQ(processor_.snapshot().velocity_status, VelocityStatus::Stopped);

  processor_.handle(InputEvent{SetVehicleStoppedEvent{false}});
  EXPECT_EQ(processor_.snapshot().velocity_status, VelocityStatus::Moving);
}

// ── SetAutowareControl ──────────────────────────────────────────────────────

TEST_F(ProcessorTest, SetAutowareControl_UpdatesState)
{
  processor_.handle(InputEvent{SetAutowareControlEvent{true}});
  EXPECT_EQ(processor_.snapshot().control_mode, ControlMode::Auto);

  processor_.handle(InputEvent{SetAutowareControlEvent{false}});
  EXPECT_EQ(processor_.snapshot().control_mode, ControlMode::Manual);
}

// ── SelfInterruption ────────────────────────────────────────────────────────

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenAutowareNotReady)
{
  const auto commands = processor_.handle(InputEvent{SelfInterruptionEvent{}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
  EXPECT_TRUE(has_effect<LogCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenNotInAutoControl)
{
  set_ready();
  // control_mode remains Unknown (not Auto)
  const auto commands = processor_.handle(InputEvent{SelfInterruptionEvent{}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
}

// ── Reset ──────────────────────────────────────────────────────────────────

TEST_F(ProcessorTest, Reset_Rejected_WhenVehicleNotStopped)
{
  processor_.handle(InputEvent{SetVehicleStoppedEvent{false}});
  const auto commands = processor_.handle(InputEvent{ResetEvent{}});
  EXPECT_FALSE(has_effect<ResetCommand>(commands));
  EXPECT_TRUE(has_effect<LogCommand>(commands));
}

TEST_F(ProcessorTest, Reset_Accepted_WhenStopped_AndAutowareNotReady)
{
  set_stopped();
  // autoware_ready is Unknown (not True) → reset accepted
  const auto commands = processor_.handle(InputEvent{ResetEvent{}});
  EXPECT_TRUE(has_effect<ResetCommand>(commands));
}

// ── SwitcherDataTimeout ─────────────────────────────────────────────────────

TEST_F(ProcessorTest, SwitcherDataTimeout_ReturnsLogWarning)
{
  const auto commands = processor_.handle(InputEvent{SwitcherDataTimeoutEvent{}});
  const auto * log = find_effect<LogCommand>(commands);
  ASSERT_NE(log, nullptr);
  EXPECT_EQ(log->level, LogLevel::Warn);
}

// ── 決定論性 ──────────────────────────────────────────────────────────────

TEST_F(ProcessorTest, SameInputSequence_ProducesSameOutput)
{
  auto run_sequence = [](Processor & p) {
    p.handle(InputEvent{SetAutowareReadyEvent{true}});
    p.handle(InputEvent{SetVehicleStoppedEvent{true}});
    p.handle(InputEvent{SetAutowareControlEvent{true}});
    return p.snapshot();
  };

  Processor p1, p2;
  const auto snap1 = run_sequence(p1);
  const auto snap2 = run_sequence(p2);

  EXPECT_EQ(snap1.autoware_ready, snap2.autoware_ready);
  EXPECT_EQ(snap1.velocity_status, snap2.velocity_status);
  EXPECT_EQ(snap1.control_mode, snap2.control_mode);
}

}  // namespace redundancy_switcher

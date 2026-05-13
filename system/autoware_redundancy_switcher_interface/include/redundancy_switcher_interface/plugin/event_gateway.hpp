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

#include "redundancy_switcher_interface/core_logic/i_processor.hpp"
#include "redundancy_switcher_interface/ir/input_events.hpp"
#include "redundancy_switcher_interface/ir/output_commands.hpp"
#include "redundancy_switcher_interface/plugin/command_bus.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// EventGateway — Adapter が使う唯一の「イベント投入口」
//
// 役割:
//   handle(event) のみを mutex で保護し、Processor の状態遷移をアトミックに行う。
//   dispatch(commands) はロック外で実行するため、Adapter の execute() 内で
//   長時間の I/O（ROS publish, UDS 送信等）が発生しても他スレッドをブロックしない。
//
// 設計上の不変条件:
//   - Adapter は gateway_->snapshot() を呼んではいけない（逆流禁止）。
//     Adapter が必要とする状態は OutputCommand に埋め込んで Processor から届ける。
//   - execute() 内で submit() を同期呼び出ししてはいけない（ロジカルに不正）。
//     タイマー callback 経由の「別の executor 起動」であれば問題ない。
// ---------------------------------------------------------------------------

class EventGateway
{
public:
  EventGateway(std::shared_ptr<IProcessor> processor, std::shared_ptr<CommandBus> command_bus)
  : processor_(processor), command_bus_(command_bus)
  {
  }

  /**
   * @brief 通知イベントを Processor に投入する（fire & forget）。
   *
   * スレッドセーフ。呼び出し元は結果を必要としない。
   * 例: SetVelocityStatusEvent, SetControlModeEvent, SetSwitcherSignalsEvent 等
   *
   * @param event 何が起きたかを表すイベント
   */
  void submit(const InputEvent & event)
  {
    std::vector<OutputCommand> commands;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands = processor_->handle(event);
    }
    if (!commands.empty()) command_bus_->dispatch(commands);
  }

  /**
   * @brief 要求イベントを Processor に投入し、判定結果を返す（request/response）。
   *
   * スレッドセーフ。dispatch() まで完了してから commands を返すため、
   * 呼び出し元は返り値から ResetResultCommand 等を取り出せる。
   * 例: ResetEvent
   *
   * @param event 何が起きたかを表すイベント
   * @return Processor が返した OutputCommand のリスト（ResetResultCommand 等を含む）
   */
  std::vector<OutputCommand> submit_request(const InputEvent & event)
  {
    std::vector<OutputCommand> commands;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands = processor_->handle(event);
    }
    if (!commands.empty()) command_bus_->dispatch(commands);
    return commands;
  }

private:
  std::shared_ptr<IProcessor> processor_;
  std::shared_ptr<CommandBus> command_bus_;
  mutable std::mutex mutex_;
};

}  // namespace redundancy_switcher

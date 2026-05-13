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

#include "redundancy_switcher_interface_base/core_logic/i_processor.hpp"
#include "redundancy_switcher_interface_base/ir/input_events.hpp"
#include "redundancy_switcher_interface_base/ir/output_commands.hpp"
#include "redundancy_switcher_interface_base/plugin/command_bus.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// EventGateway — Adapter が使う唯一の「イベント投入口」
//
// 役割:
//   handle(event) + command_bus.dispatch(commands) をひとつの mutex で保護し、
//   MultiThreadedExecutor 環境でも Processor の状態遷移とエフェクト配信が
//   アトミックに行われることを保証する。
//
// 設計上の注意:
//   execute() 内で submit() を同期的に呼び出してはいけない（デッドロックになる）。
//   タイマー callback のように "別の executor 呼び出し" 経由なら問題ない。
// ---------------------------------------------------------------------------

class EventGateway
{
public:
  EventGateway(std::shared_ptr<IProcessor> processor, std::shared_ptr<CommandBus> command_bus)
  : processor_(processor), command_bus_(command_bus)
  {
  }

  /**
   * @brief InputEvent を Processor に投入し、エフェクトを CommandBus に配信する。
   *
   * スレッドセーフ。handle() → dispatch() はひとつの mutex で保護される。
   *
   * @param event 何が起きたかを表すイベント
   */
  void submit(const InputEvent & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto commands = processor_->handle(event);
    if (!commands.empty()) command_bus_->dispatch(commands);
  }

  /**
   * @brief 現在のドメイン状態スナップショットを返す（診断用）。
   *
   * スレッドセーフ。
   */
  DomainSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return processor_->snapshot();
  }

private:
  std::shared_ptr<IProcessor> processor_;
  std::shared_ptr<CommandBus> command_bus_;
  mutable std::mutex mutex_;
};

}  // namespace redundancy_switcher

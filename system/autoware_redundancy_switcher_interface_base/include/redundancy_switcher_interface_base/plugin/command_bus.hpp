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

#include "redundancy_switcher_interface_base/ir/output_commands.hpp"
#include "redundancy_switcher_interface_base/plugin/i_adapter_plugin.hpp"

#include <algorithm>
#include <memory>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// CommandBus — OutputCommand を全登録 Adapter にブロードキャストする
//
// 役割:
//   - Adapter が processor->handle(event) で取得した OutputCommands を
//     dispatch() に渡すと、登録済みの全 Adapter の execute() が呼ばれる。
//   - 各 Adapter は担当エフェクトのみ処理し、担当外は無視する。
//   - Adapter は weak_ptr で登録されるため、ライフタイム管理はノード側が行う。
// ---------------------------------------------------------------------------

class CommandBus
{
public:
  /**
   * @brief エフェクトを受け取る Adapter を登録する。
   * @param adapter  execute() が呼ばれる Adapter
   */
  void add_handler(std::weak_ptr<IAdapterPlugin> adapter) { handlers_.push_back(adapter); }

  /**
   * @brief OutputCommand のリストを全登録 Adapter にブロードキャストする。
   *
   * 期限切れ (expired) な weak_ptr は自動的に除去される。
   *
   * @param commands  Processor が返した OutputCommand のリスト
   */
  void dispatch(const std::vector<OutputCommand> & commands)
  {
    // 期限切れを除去しながら処理
    handlers_.erase(
      std::remove_if(
        handlers_.begin(), handlers_.end(),
        [](const std::weak_ptr<IAdapterPlugin> & wp) { return wp.expired(); }),
      handlers_.end());

    for (const auto & command : commands) {
      for (auto & wp : handlers_) {
        if (auto adapter = wp.lock()) {
          adapter->execute(command);
        }
      }
    }
  }

private:
  std::vector<std::weak_ptr<IAdapterPlugin>> handlers_;
};

}  // namespace redundancy_switcher

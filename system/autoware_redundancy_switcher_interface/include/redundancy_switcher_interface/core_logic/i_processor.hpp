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
#include "redundancy_switcher_interface/ir/input_events.hpp"
#include "redundancy_switcher_interface/ir/output_commands.hpp"

#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// IProcessor — Processor の公開インターフェース
//
// 設計上の制約:
//   - Adapter / ROS / UDS / 通信詳細を一切知らない。
//   - 同じ InputEvent 列を与えれば常に同じ OutputCommand 列と状態を返す。
//   - 例外を外に漏らさない (エラーは OutputCommand か DomainSnapshot で表現)。
//   - スレッドセーフ保証は持たない (呼び出し側 Adapter が保証する)。
// ---------------------------------------------------------------------------

class IProcessor
{
public:
  virtual ~IProcessor() = default;

  /**
   * @brief イベントを受け取り、実行すべきエフェクトのリストを返す。
   *
   * Processor の唯一の公開エントリポイント。
   * 内部状態を更新し、外部に実行させたいエフェクトを返す。
   * Processor 自身は何も実行しない。
   *
   * @param event  何が起きたかを表す InputEvent
   * @return       何をすべきかを表す OutputCommand のリスト
   */
  virtual std::vector<OutputCommand> handle(const InputEvent & event) = 0;

  /**
   * @brief 現在のドメイン状態のスナップショットを返す。
   *
   * 診断・テスト・ロギング目的で使用する。
   * この呼び出しは状態を変更しない。
   */
  virtual DomainSnapshot snapshot() const = 0;
};

}  // namespace redundancy_switcher

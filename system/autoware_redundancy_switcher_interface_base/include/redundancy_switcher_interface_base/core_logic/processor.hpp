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

#include <vector>

namespace redundancy_switcher
{

/**
 * @brief デフォルト Processor 実装
 *
 * 冗長系切り替え判定とドメイン状態管理を行う。
 * ROS 2 / UDS / 通信詳細に一切依存しない純粋な C++ クラス。
 *
 * 拡張方法:
 *   独自ドメインロジックが必要な場合は IProcessor を直接継承し、
 *   handle() をオーバーライドする。
 *
 * 設計上の制約:
 *   - handle() はスレッドセーフを保証しない。呼び出し側 (Adapter) が保証する。
 *   - handle() は例外を外に漏らさない。エラーは OutputCommand で表現する。
 *   - Adapter/Port への直接参照は持たない。
 */
class Processor : public IProcessor
{
public:
  Processor();

  /**
   * @brief InputEvent を処理し、実行すべき OutputCommand のリストを返す。
   *
   * 内部状態を更新し、Adapter に実行させたいエフェクトを決定して返す。
   * Processor 自身は何も実行しない。
   */
  std::vector<OutputCommand> handle(const InputEvent & event) override;

  /**
   * @brief 現在のドメイン状態スナップショットを返す。状態は変更しない。
   */
  DomainSnapshot snapshot() const override { return state_; }

private:
  // ── ドメイン状態 ──────────────────────────────────────────────────────
  DomainSnapshot state_;

  // ── イベント別ハンドラ ─────────────────────────────────────────────────
  std::vector<OutputCommand> self_interruption(const SelfInterruptionEvent & e);
  std::vector<OutputCommand> reset(const ResetEvent & e);
  std::vector<OutputCommand> set_autoware_ready(const SetAutowareReadyEvent & e);
  std::vector<OutputCommand> set_vehicle_stopped(const SetVehicleStoppedEvent & e);
  std::vector<OutputCommand> set_autoware_control(const SetAutowareControlEvent & e);
  std::vector<OutputCommand> switcher_data_timeout(const SwitcherDataTimeoutEvent & e);
};

}  // namespace redundancy_switcher

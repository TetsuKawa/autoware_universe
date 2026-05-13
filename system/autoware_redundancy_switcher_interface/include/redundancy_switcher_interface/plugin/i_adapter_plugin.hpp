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

#include "redundancy_switcher_interface/ir/output_commands.hpp"

#include <memory>

// rclcpp::Node は前方宣言のみ。具体的な Adapter 実装でフルインクルードすること。
// これにより i_adapter_plugin.hpp / command_bus.hpp / event_gateway.hpp が
// ROS に依存しなくなり、Processor・EventGateway の純粋 GTest が書けるようになる。
namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace redundancy_switcher
{

// Forward declarations to avoid circular include
class CommandBus;
class EventGateway;

// ---------------------------------------------------------------------------
// IAdapterPlugin — Adapter プラグインの基底インターフェース
//
// Adapter の責務:
//   1. 外部 I/O (ROS topic/service) を受信し InputEvent に変換
//   2. gateway_->submit(event) で Processor に投入（スレッドセーフ）
//   3. execute(command) で CommandBus から配信されたエフェクトを実行
//      ※ 担当外のエフェクト型は std::visit の default case で無視すること
//
// execute() 内での制約:
//   - gateway_->submit() を同期的に呼び出してはいけない（デッドロック）
//   - タイマー callback 経由など「別の executor 起動」なら問題ない
// ---------------------------------------------------------------------------

class IAdapterPlugin
{
public:
  virtual ~IAdapterPlugin() = default;

  /**
   * @brief ROS リソースの初期化と EventGateway の接続。
   *
   * @param node     ROS ノードポインタ（サブスク/パブリッシャー確保用）
   * @param gateway  スレッドセーフなイベント投入口
   */
  virtual void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) = 0;

  /**
   * @brief Processor が返した OutputCommand を実行する。
   *
   * CommandBus から全 Adapter に対して呼ばれる。
   * 自分が担当するエフェクト型のみ処理し、担当外は無視すること。
   */
  virtual void execute(const OutputCommand & command) = 0;
};

}  // namespace redundancy_switcher

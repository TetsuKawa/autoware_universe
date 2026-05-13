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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/ir/domain_types.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>
#include <optional>

namespace redundancy_switcher
{

/**
 * @brief 集約ステータス診断を publish する組み込み Adapter
 *
 * 責任:
 *   - UpdateStatusDiagCommand を受け取り diagnostic_updater をトリガーする
 *   - DomainSnapshot の各状態（SwitcherSignals/AutowareReady/VelocityStatus/ControlMode）
 *     を集約して redundancy_switcher_interface/status として publish する
 *   - 過渡状態（is_stable/is_self_interrupted/is_faulted が全て false）が
 *     diag.transitional_timeout_milli を超えて継続した場合 ERROR を出力する
 *   - 特定 Switcher 実装・ECU アーキテクチャに依存しない（アーキテクチャ非依存の共通 diag）
 *   - EventGateway には何も投入しない（純粋な出力専用）
 */
class DiagAdapter : public IAdapterPlugin
{
public:
  DiagAdapter() = default;
  ~DiagAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  void update_status(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Node * node_{nullptr};
  std::unique_ptr<diagnostic_updater::Updater> updater_;
  std::optional<DomainSnapshot> last_snapshot_;  // execute() で受け取った最新スナップショット
  double transitional_timeout_milli_{10000.0};
  std::optional<rclcpp::Time> stamp_transitional_start_;
};

}  // namespace redundancy_switcher

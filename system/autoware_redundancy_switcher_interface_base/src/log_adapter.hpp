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

#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface_base/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface_base/plugin/i_adapter_plugin.hpp>

#include <memory>

namespace redundancy_switcher
{

/**
 * @brief LogCommand のみを処理するロギング専用 Adapter
 *
 * 責任:
 *   - LogCommand を受け取り RCLCPP_* マクロでログ出力する
 *   - それ以外の OutputCommand はすべて無視する
 *   - EventGateway には何も投入しない（純粋な出力専用）
 */
class LogAdapter : public IAdapterPlugin
{
public:
  LogAdapter() = default;
  ~LogAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;

  void execute(const OutputCommand & command) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("redundancy_switcher")};
};

}  // namespace redundancy_switcher

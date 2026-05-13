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

#include <nlohmann/json.hpp>

#include <cstdint>
#include <vector>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// Interface → Switcher  (送信)
// self_interruption: Autoware 側から自己故障を申告する
// reset:             リセット要求
// ---------------------------------------------------------------------------
struct ElectionRequest
{
  bool self_interruption{false};
  bool reset{false};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ElectionRequest, self_interruption, reset)

// ---------------------------------------------------------------------------
// Switcher → Interface  (受信)
// active_unit_ids: 現在アクティブな制御ユニット ID 一覧 (tmp_msgs/ActiveControlUnit.ids)
// ---------------------------------------------------------------------------
struct ElectionStatus
{
  uint8_t node_state{0};
  uint8_t node_id{0};
  uint8_t leader_id{0};
  uint8_t path_info{0};
  bool main_ecu_to_main_ecu_connected{false};
  bool main_ecu_to_sub_ecu_connected{false};
  bool main_ecu_to_main_vcu_connected{false};
  bool main_ecu_to_sub_vcu_connected{false};
  bool sub_ecu_to_main_ecu_connected{false};
  bool sub_ecu_to_sub_ecu_connected{false};
  bool sub_ecu_to_main_vcu_connected{false};
  bool sub_ecu_to_sub_vcu_connected{false};
  bool main_vcu_to_main_ecu_connected{false};
  bool main_vcu_to_sub_ecu_connected{false};
  bool main_vcu_to_main_vcu_connected{false};
  bool main_vcu_to_sub_vcu_connected{false};
  bool sub_vcu_to_main_ecu_connected{false};
  bool sub_vcu_to_sub_ecu_connected{false};
  bool sub_vcu_to_main_vcu_connected{false};
  bool sub_vcu_to_sub_vcu_connected{false};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ElectionStatus, node_state, node_id, leader_id, path_info, main_ecu_to_main_ecu_connected,
  main_ecu_to_sub_ecu_connected, main_ecu_to_main_vcu_connected, main_ecu_to_sub_vcu_connected,
  sub_ecu_to_main_ecu_connected, sub_ecu_to_sub_ecu_connected, sub_ecu_to_main_vcu_connected,
  sub_ecu_to_sub_vcu_connected, main_vcu_to_main_ecu_connected, main_vcu_to_sub_ecu_connected,
  main_vcu_to_main_vcu_connected, main_vcu_to_sub_vcu_connected, sub_vcu_to_main_ecu_connected,
  sub_vcu_to_sub_ecu_connected, sub_vcu_to_main_vcu_connected, sub_vcu_to_sub_vcu_connected)

}  // namespace redundancy_switcher

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

#include "service_divider.hpp"

#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <optional>
#include <vector>

namespace autoware::redundancy_service_divider
{

ServiceDivider::ServiceDivider(
  rclcpp::Node & node,
  int service_timeout_ms,
  bool is_redundant,
  ClientPair<ResetRedundancySwitcher> reset_switcher_pair)
: node_(node),
  service_timeout_ms_(service_timeout_ms),
  is_redundant_(is_redundant),
  clients_reset_redundancy_switcher_(reset_switcher_pair)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_change_operation_mode_ = node_.create_service<ChangeOperationMode>(
    "~/input/change_operation_mode",
    std::bind(&ServiceDivider::on_change_operation_mode, this, _1, _2));
  srv_change_autoware_control_ = node_.create_service<ChangeAutowareControl>(
    "~/input/change_autoware_control",
    std::bind(&ServiceDivider::on_change_autoware_control, this, _1, _2));
  srv_reset_mrm_ = node_.create_service<ResetMrm>(
    "~/input/reset_mrm",
    std::bind(&ServiceDivider::on_reset_mrm, this, _1, _2));

  clients_change_operation_mode_   = make_client_pair<ChangeOperationMode>(node_, "change_operation_mode", is_redundant_);
  clients_change_autoware_control_ = make_client_pair<ChangeAutowareControl>(node_, "change_autoware_control", is_redundant_);
  clients_reset_diag_graph_        = make_client_pair<ResetDiagGraph>(node_, "reset_diag_graph", is_redundant_);
  // clients_reset_redundancy_switcher_ is set via constructor (shared with AutowareReadyDivider)
}

void ServiceDivider::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr request,
  ChangeOperationMode::Response::SharedPtr response)
{
  dispatch(clients_change_operation_mode_, request, response,
           "change_operation_mode", ResponseStatus::SERVICE_TIMEOUT);
}

void ServiceDivider::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr request,
  ChangeAutowareControl::Response::SharedPtr response)
{
  dispatch(clients_change_autoware_control_, request, response,
           "change_autoware_control", ResponseStatus::SERVICE_TIMEOUT);
}

void ServiceDivider::on_reset_mrm(
  ResetMrm::Request::SharedPtr /*request*/,
  ResetMrm::Response::SharedPtr response)
{
  using ExtApi = tier4_external_api_msgs::msg::ResponseStatus;

  const auto extract = [](auto r) -> CallResult {
    return {r->status.success, static_cast<int>(r->status.code), r->status.message};
  };
  const int timeout_code = static_cast<int>(ResponseStatus::SERVICE_TIMEOUT);

  // Send requests simultaneously before waiting.
  // When not redundant: only reset_diag_graph/main. Sub and reset_redundancy_switcher are skipped.
  auto f_diag_main = send_request<ResetDiagGraph>(
    clients_reset_diag_graph_.main, std::make_shared<ResetDiagGraph::Request>());
  auto f_diag_sub = is_redundant_
    ? std::make_optional(send_request<ResetDiagGraph>(clients_reset_diag_graph_.sub, std::make_shared<ResetDiagGraph::Request>()))
    : std::nullopt;
  auto f_switcher_main = is_redundant_
    ? std::make_optional(send_request<ResetRedundancySwitcher>(clients_reset_redundancy_switcher_.main, std::make_shared<ResetRedundancySwitcher::Request>()))
    : std::nullopt;
  auto f_switcher_sub = is_redundant_
    ? std::make_optional(send_request<ResetRedundancySwitcher>(clients_reset_redundancy_switcher_.sub, std::make_shared<ResetRedundancySwitcher::Request>()))
    : std::nullopt;

  std::vector<CallResult> results;
  results.push_back(collect_result<ResetDiagGraph>(f_diag_main, "reset_diag_graph [main]", extract, timeout_code));
  if (f_diag_sub)      results.push_back(collect_result<ResetDiagGraph>(         *f_diag_sub,      "reset_diag_graph [sub]",            extract, timeout_code));
  if (f_switcher_main) results.push_back(collect_result<ResetRedundancySwitcher>(*f_switcher_main,  "reset_redundancy_switcher [main]",  extract, timeout_code));
  if (f_switcher_sub)  results.push_back(collect_result<ResetRedundancySwitcher>(*f_switcher_sub,   "reset_redundancy_switcher [sub]",   extract, timeout_code));

  const auto * failed = [&]() -> const CallResult * {
    for (const auto & r : results) {
      if (!r.success) return &r;
    }
    return nullptr;
  }();

  if (!failed) {
    response->status.code    = ExtApi::SUCCESS;
    response->status.message = "";
  } else {
    response->status.code    = ExtApi::ERROR;
    response->status.message = failed->message;
  }
}

}  // namespace autoware::redundancy_service_divider

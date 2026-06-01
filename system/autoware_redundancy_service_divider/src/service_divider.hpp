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

#ifndef SERVICE_DIVIDER_HPP_
#define SERVICE_DIVIDER_HPP_

#include "client_pair.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <tier4_external_api_msgs/srv/reset_mrm.hpp>
#include <tier4_system_msgs/srv/reset_diag_graph.hpp>
#include <tier4_system_msgs/srv/reset_redundancy_switcher.hpp>

#include <chrono>
#include <optional>
#include <string>

namespace autoware::redundancy_service_divider
{

// Receives external service requests and forwards them to both main and sub.
// Success is reported only when both respond successfully within the timeout.
class ServiceDivider
{
public:
  // reset_switcher_pair is shared with AutowareReadyDivider to avoid duplicate client objects.
  ServiceDivider(
    rclcpp::Node & node,
    int service_timeout_ms,
    bool is_redundant,
    ClientPair<tier4_system_msgs::srv::ResetRedundancySwitcher> reset_switcher_pair);

private:
  using ChangeOperationMode     = autoware_system_msgs::srv::ChangeOperationMode;
  using ChangeAutowareControl   = autoware_system_msgs::srv::ChangeAutowareControl;
  using ResetMrm                = tier4_external_api_msgs::srv::ResetMrm;
  using ResetDiagGraph          = tier4_system_msgs::srv::ResetDiagGraph;
  using ResetRedundancySwitcher = tier4_system_msgs::srv::ResetRedundancySwitcher;
  using ResponseStatus          = autoware_common_msgs::msg::ResponseStatus;

  rclcpp::Node & node_;
  int service_timeout_ms_;
  bool is_redundant_;

  // Input services
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_operation_mode_;
  rclcpp::Service<ChangeAutowareControl>::SharedPtr srv_change_autoware_control_;
  rclcpp::Service<ResetMrm>::SharedPtr srv_reset_mrm_;

  // Output client pairs (main / sub)
  ClientPair<ChangeOperationMode>     clients_change_operation_mode_;
  ClientPair<ChangeAutowareControl>   clients_change_autoware_control_;
  ClientPair<ResetDiagGraph>          clients_reset_diag_graph_;
  ClientPair<ResetRedundancySwitcher> clients_reset_redundancy_switcher_;

  // Service callbacks
  void on_change_operation_mode(
    ChangeOperationMode::Request::SharedPtr request,
    ChangeOperationMode::Response::SharedPtr response);
  void on_change_autoware_control(
    ChangeAutowareControl::Request::SharedPtr request,
    ChangeAutowareControl::Response::SharedPtr response);
  // Triggers reset_diag_graph and reset_redundancy_switcher on all four clients.
  void on_reset_mrm(
    ResetMrm::Request::SharedPtr request,
    ResetMrm::Response::SharedPtr response);

  // Dispatch helpers
  struct CallResult {
    bool        success{false};
    int         code{0};
    std::string message;
  };

  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedFuture send_request(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request)
  {
    return client->async_send_request(request).future.share();
  }

  template <typename ServiceT, typename ExtractFn>
  CallResult collect_result(
    typename rclcpp::Client<ServiceT>::SharedFuture future,
    const std::string & label,
    ExtractFn extract,
    int timeout_code)
  {
    if (
      future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) ==
      std::future_status::ready) {
      const auto result = extract(future.get());
      if (!result.success) {
        RCLCPP_ERROR(node_.get_logger(), "%s service failed", label.c_str());
      }
      return result;
    }
    RCLCPP_WARN(node_.get_logger(), "%s service timeout", label.c_str());
    return {false, timeout_code, label + " service timeout"};
  }

  // Sends request to both main and sub, collects results, and writes the merged outcome
  // back into response->status. Works with any service whose response has
  // .status.success / .code / .message fields.
  template <typename ServiceT>
  void dispatch(
    ClientPair<ServiceT> & clients,
    typename ServiceT::Request::SharedPtr request,
    typename ServiceT::Response::SharedPtr response,
    const std::string & label,
    int timeout_code)
  {
    const auto extract = [](typename ServiceT::Response::SharedPtr r) -> CallResult {
      return {r->status.success, static_cast<int>(r->status.code), r->status.message};
    };

    auto f_main = send_request<ServiceT>(
      clients.main, std::make_shared<typename ServiceT::Request>(*request));
    // Send sub before waiting for main to keep both in-flight simultaneously.
    auto f_sub = is_redundant_
      ? std::make_optional(send_request<ServiceT>(clients.sub, std::make_shared<typename ServiceT::Request>(*request)))
      : std::nullopt;

    const auto r_main = collect_result<ServiceT>(f_main, "Main " + label, extract, timeout_code);

    if (!f_sub) {
      response->status.success = r_main.success;
      response->status.code    = static_cast<decltype(response->status.code)>(r_main.code);
      response->status.message = r_main.message;
      return;
    }

    const auto r_sub = collect_result<ServiceT>(*f_sub, "Sub " + label, extract, timeout_code);

    const auto & failed = r_main.success ? r_sub : r_main;
    response->status.success = r_main.success && r_sub.success;
    response->status.code    = static_cast<decltype(response->status.code)>(
      response->status.success ? r_main.code : failed.code);
    response->status.message = response->status.success ? r_main.message : failed.message;
  }
};

}  // namespace autoware::redundancy_service_divider

#endif  // SERVICE_DIVIDER_HPP_

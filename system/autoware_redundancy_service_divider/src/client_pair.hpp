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

#ifndef CLIENT_PAIR_HPP_
#define CLIENT_PAIR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>
#include <string>

namespace autoware::redundancy_service_divider
{

// Holds a main/sub client pair for one service type.
// Keep callback groups alongside clients to ensure their lifetime.
template <typename ServiceT>
struct ClientPair {
  rclcpp::CallbackGroup::SharedPtr main_group;
  rclcpp::CallbackGroup::SharedPtr sub_group;  // nullptr when create_sub=false
  typename rclcpp::Client<ServiceT>::SharedPtr main;
  typename rclcpp::Client<ServiceT>::SharedPtr sub;  // nullptr when create_sub=false
};

// Creates a client pair under "~/output/<service_name>/{main,sub}".
// Pass create_sub=false (e.g. when is_redundant=false) to skip allocating the sub client.
template <typename ServiceT>
ClientPair<ServiceT> make_client_pair(
  rclcpp::Node & node, const std::string & service_name, bool create_sub = true)
{
  ClientPair<ServiceT> pair;
  pair.main_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pair.main = node.create_client<ServiceT>(
    "~/output/" + service_name + "/main", rmw_qos_profile_services_default,
    pair.main_group);
  if (create_sub) {
    pair.sub_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pair.sub = node.create_client<ServiceT>(
      "~/output/" + service_name + "/sub", rmw_qos_profile_services_default,
      pair.sub_group);
  }
  return pair;
}

// Sends request to both main and sub simultaneously, waits for both responses,
// and returns true only if both succeed.
// extract_success: (Response::SharedPtr) -> bool
template <typename ServiceT, typename ExtractSuccessFn>
bool call_client_pair(
  ClientPair<ServiceT> & clients,
  typename ServiceT::Request::SharedPtr request,
  const char * label,
  int timeout_ms,
  const rclcpp::Logger & logger,
  ExtractSuccessFn extract_success,
  bool call_sub = true)
{
  // Send both before waiting to keep requests in-flight simultaneously.
  auto f_main = clients.main->async_send_request(request).future.share();
  auto f_sub  = call_sub
    ? std::make_optional(clients.sub->async_send_request(request).future.share())
    : std::nullopt;

  bool ok = true;
  const auto check = [&](auto & future, const char * side) {
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
      RCLCPP_ERROR(logger, "%s [%s] service timeout", label, side);
      ok = false;
    } else if (!extract_success(future.get())) {
      RCLCPP_ERROR(logger, "%s [%s] service failed", label, side);
      ok = false;
    }
  };
  check(f_main, "main");
  if (f_sub) check(*f_sub, "sub");
  return ok;
}

}  // namespace autoware::redundancy_service_divider

#endif  // CLIENT_PAIR_HPP_

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
#include "log_adapter.hpp"

#include <memory>
#include <stdexcept>

namespace redundancy_switcher
{

// ---------------------------------------------------------------------------
// overloaded helper (C++17)
// ---------------------------------------------------------------------------

template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------

void LogAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> /*gateway*/)
{
  if (!node) {
    throw std::invalid_argument("LogAdapter: node is null");
  }
  logger_ = node->get_logger();
}

// ---------------------------------------------------------------------------
// execute — LogCommand のみ処理し、他はすべて無視
// ---------------------------------------------------------------------------

void LogAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const LogCommand & c) {
        switch (c.level) {
          case LogLevel::Debug:
            RCLCPP_DEBUG(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Info:
            RCLCPP_INFO(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Warn:
            RCLCPP_WARN(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Error:
            RCLCPP_ERROR(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Fatal:
            RCLCPP_FATAL(logger_, "%s", c.message.c_str());
            break;
        }
      },
      [](const auto &) { /* 他 Adapter の担当 — 無視 */ }},
    command);
}

}  // namespace redundancy_switcher

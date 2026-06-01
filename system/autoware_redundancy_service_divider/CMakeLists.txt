cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_service_divider)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/service_divider.cpp
  src/autoware_ready_divider.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::redundancy_service_divider::RedundancyServiceDivider"
  EXECUTABLE autoware_redundancy_service_divider_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

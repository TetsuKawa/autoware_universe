cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_switcher_interface_base)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/processor.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/log_adapter.cpp
  src/redundancy_switcher_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "redundancy_switcher::RedundancySwitcherInterface"
  EXECUTABLE redundancy_switcher_interface_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Processor の純粋ドメインロジックテスト（ROS 依存なし）
  ament_add_gtest(test_processor test/test_processor.cpp)
  target_link_libraries(test_processor ${PROJECT_NAME}_core)

  # EventGateway + CommandBus のテスト（ROS 依存なし）
  ament_add_gtest(test_event_gateway test/test_event_gateway.cpp)
  target_link_libraries(test_event_gateway ${PROJECT_NAME}_core)
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)

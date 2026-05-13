cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_switcher_interface_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(nlohmann_json REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/switcher_adapter.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

pluginlib_export_plugin_description_file(autoware_redundancy_switcher_interface plugins.xml)

ament_auto_package(INSTALL_TO_SHARE config)

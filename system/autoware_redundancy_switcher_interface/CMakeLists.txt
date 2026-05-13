cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_switcher_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/subsystem_adapter.cpp
  src/switcher_adapter.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
)

pluginlib_export_plugin_description_file(autoware_redundancy_switcher_interface_base plugins.xml)

ament_auto_package()

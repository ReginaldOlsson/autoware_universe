cmake_minimum_required(VERSION 3.14)
project(autoware_moveit_freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(moveit_freespace_planner_node SHARED
  src/freespace_planner_node.cpp
  src/utils.cpp
)

rclcpp_components_register_node(moveit_freespace_planner_node
  PLUGIN "autoware::moveit_freespace_planner::FreespacePlannerNode"
  EXECUTABLE moveit_freespace_planner
)

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

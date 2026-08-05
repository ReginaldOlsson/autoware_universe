cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(freespace_planner_node SHARED
  src/autoware_freespace_planner/freespace_planner_node.cpp
  src/autoware_freespace_planner/utils.cpp
)

# Enable astar_cuda when the algorithms package installed the CUDA header
find_file(_astar_cuda_hdr
  NAMES autoware/freespace_planning_algorithms/astar_search_cuda.hpp
  PATHS ${autoware_freespace_planning_algorithms_INCLUDE_DIRS}
  NO_DEFAULT_PATH
)
if(_astar_cuda_hdr)
  target_compile_definitions(freespace_planner_node PRIVATE AUTOWARE_CUDA_ENABLED)
  message(STATUS "autoware_freespace_planner: enabling astar_cuda backend")
endif()

rclcpp_components_register_node(freespace_planner_node
  PLUGIN "autoware::freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_freespace_planner_node_interface.cpp
    test/test_freespace_planner_utils.cpp
    test/test_freespace_planner.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    freespace_planner_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

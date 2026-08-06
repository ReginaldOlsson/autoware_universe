cmake_minimum_required(VERSION 3.14)
project(autoware_moveit_freespace_planning_algorithms)

find_package(autoware_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/moveit_ackermann_planner.cpp
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(${PROJECT_NAME} yaml-cpp)
endif()

ament_auto_package()

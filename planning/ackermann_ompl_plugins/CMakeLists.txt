cmake_minimum_required(VERSION 3.8)
project(ackermann_ompl_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Core library used by Autoware AbstractPlanningAlgorithm adapter (makePlan path)
add_library(${PROJECT_NAME} SHARED
  src/ackermann_state_space.cpp
  src/ackermann_ompl_solver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  Eigen3
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

if(TARGET ompl::ompl)
  target_link_libraries(${PROJECT_NAME} ompl::ompl)
elseif(TARGET ompl)
  target_link_libraries(${PROJECT_NAME} ompl)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${OMPL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})
endif()

# Optional MoveIt plugins: enable with -DACKERMANN_BUILD_MOVEIT_PLUGINS=ON after installing MoveIt
option(ACKERMANN_BUILD_MOVEIT_PLUGINS "Build MoveIt PlannerManager plugins" OFF)
if(ACKERMANN_BUILD_MOVEIT_PLUGINS)
  find_package(moveit_core REQUIRED)
  find_package(pluginlib REQUIRED)
  target_sources(${PROJECT_NAME} PRIVATE
    src/ackermann_validity_checker.cpp
    src/reeds_shepp_planning_context.cpp
    src/reeds_shepp_planner_manager.cpp
  )
  ament_target_dependencies(${PROJECT_NAME} moveit_core pluginlib)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ACKERMANN_OMPL_HAS_MOVEIT=1)
  pluginlib_export_plugin_description_file(moveit_core ackermann_ompl_plugins.xml)
  install(FILES ackermann_ompl_plugins.xml DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ompl Eigen3 tf2 tf2_geometry_msgs geometry_msgs nav_msgs)
ament_export_include_directories(include)

ament_package()

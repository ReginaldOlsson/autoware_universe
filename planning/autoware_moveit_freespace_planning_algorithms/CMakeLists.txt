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

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}-test
    test/test_moveit_ackermann_planner.cpp
  )
  set_tests_properties(${PROJECT_NAME}-test PROPERTIES TIMEOUT 60)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )
endif()

ament_auto_package()

cmake_minimum_required(VERSION 3.14)
project(lie_integrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lie_algebra.cpp
  src/terrain.cpp
  src/lie_integrator_3d.cpp
  src/lie_integrator_node.cpp
  src/pure_imu_odometry.cpp
  src/pure_imu_node.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lie_integrator::LieIntegratorNode"
  EXECUTABLE lie_integrator_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lie_integrator::PureImuNode"
  EXECUTABLE pure_imu_odometry_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_add_executable(lie_integrator_demo
  src/lie_integrator_demo.cpp
)
target_link_libraries(lie_integrator_demo
  ${PROJECT_NAME}
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)

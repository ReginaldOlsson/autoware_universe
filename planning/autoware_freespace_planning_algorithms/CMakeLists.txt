cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planning_algorithms)

find_package(autoware_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(CUDA)

autoware_package()
ament_python_install_package(${PROJECT_NAME})

ament_auto_add_library(reeds_shepp SHARED
  src/reeds_shepp.cpp
)

ament_auto_add_library(rrtstar_core SHARED
  src/rrtstar_core.cpp
)

target_link_libraries(rrtstar_core
  reeds_shepp
)

set(FREESPACE_PLANNING_SOURCES
  src/abstract_algorithm.cpp
  src/astar_search.cpp
  src/rrtstar.cpp
)

set(CUDA_ENABLED FALSE)
if(CUDA_FOUND)
  set(CUDA_ENABLED TRUE)
  message(STATUS "CUDA found: building AstarSearchCuda")
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
  # Ada (RTX 40) and Ampere; avoid arches newer than the installed nvcc
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
  list(APPEND CUDA_NVCC_FLAGS "--extended-lambda")
  include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
  # Prefer gcc-12 when available (CUDA 12.0 host compiler limit)
  if(EXISTS /usr/bin/g++-12)
    set(CUDA_HOST_COMPILER /usr/bin/g++-12)
    set(CMAKE_CUDA_HOST_COMPILER /usr/bin/g++-12)
  else()
    list(APPEND CUDA_NVCC_FLAGS "-allow-unsupported-compiler")
  endif()

  cuda_add_library(${PROJECT_NAME}_cuda_kernels SHARED
    src/cuda/astar_cuda_kernels.cu
  )
  target_include_directories(${PROJECT_NAME}_cuda_kernels PUBLIC include)
  target_link_libraries(${PROJECT_NAME}_cuda_kernels ${CUDA_LIBRARIES})
  install(TARGETS ${PROJECT_NAME}_cuda_kernels
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  ament_auto_add_library(${PROJECT_NAME}_cuda SHARED
    src/cuda/astar_search_cuda.cpp
  )
  target_compile_definitions(${PROJECT_NAME}_cuda PUBLIC AUTOWARE_CUDA_ENABLED)
  target_link_libraries(${PROJECT_NAME}_cuda
    reeds_shepp
    ${PROJECT_NAME}_cuda_kernels
    ${CUDA_LIBRARIES}
  )
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${FREESPACE_PLANNING_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  reeds_shepp
)

if(CUDA_ENABLED)
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_cuda
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC AUTOWARE_CUDA_ENABLED)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}-test
    test/src/test_freespace_planning_algorithms.cpp
  )
  set_tests_properties(${PROJECT_NAME}-test PROPERTIES TIMEOUT 120)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )

  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(rrtstar_core_informed-test
    test/src/test_rrtstar_core_informed.cpp
  )
  target_link_libraries(rrtstar_core_informed-test
    ${PROJECT_NAME}
  )

  if(CUDA_ENABLED)
    ament_add_ros_isolated_gtest(test_astar_cuda_correctness
      test/src/test_astar_cuda_correctness.cpp
    )
    set_tests_properties(test_astar_cuda_correctness PROPERTIES TIMEOUT 180)
    target_link_libraries(test_astar_cuda_correctness
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cuda
    )

    add_executable(astar_cpu_vs_cuda_bench
      test/src/bench_astar_cpu_vs_cuda.cpp
    )
    target_link_libraries(astar_cpu_vs_cuda_bench
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cuda
    )
    install(TARGETS astar_cpu_vs_cuda_bench
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)

install(PROGRAMS
  test/debug_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-optimization-argument")

pybind11_add_module(${PROJECT_NAME}_pybind SHARED
  scripts/bind/astar_search_pybind.cpp
)

include_directories(${PROJECT_NAME}_pybind PRIVATE
  include/
)
target_link_libraries(${PROJECT_NAME}_pybind PRIVATE
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_pybind
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

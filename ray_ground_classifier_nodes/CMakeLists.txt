# Copyright 2017-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(use_OMP FALSE)
if(use_OMP)
  find_package(OpenMP REQUIRED)
  set(OpenMP_LIBS gomp)
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
else()
  set(OpenMP_FLAGS "-Wno-unknown-pragmas")
endif()


## dependencies
set(PROJECT_CATKIN_DEPS
    roscpp
    # pcl_conversions
    pcl_ros
    tf
    # visualization_msgs
    ray_ground_classifier
    # autoware_auto_common
    # autoware_auto_geometry
    # autoware_auto_algorithm
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPS}
  DEPENDS
    # OpenCV
)

add_definitions(${catkin_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ## dependencies
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

# ### Build cloud node as library
# set(CLOUD_NODE_LIB ray_ground_classifier_cloud_node)
# ament_auto_add_library(${CLOUD_NODE_LIB} SHARED
#   include/ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp
#   include/ray_ground_classifier_nodes/visibility_control.hpp
#   src/ray_ground_classifier_cloud_node.cpp)
# autoware_set_compile_options(${CLOUD_NODE_LIB})

# rclcpp_components_register_node(${CLOUD_NODE_LIB}
#   PLUGIN "autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode"
#   EXECUTABLE ${CLOUD_NODE_LIB}_exe
# )

# target_link_libraries(${CLOUD_NODE_LIB} ${OpenMP_LIBS})
# target_compile_options(${CLOUD_NODE_LIB} PRIVATE ${OpenMP_FLAGS})

# if(BUILD_TESTING)
#   # run linters
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_ray_ground_classifier_nodes_gtest
#     test/src/test_ray_ground_classifier_nodes.cpp
#   )
#   autoware_set_compile_options(test_ray_ground_classifier_nodes_gtest)
#   target_compile_options(test_ray_ground_classifier_nodes_gtest PRIVATE -Wno-conversion)
#   target_include_directories(test_ray_ground_classifier_nodes_gtest
#     PRIVATE "test/include" "include"
#   )
#   target_link_libraries(test_ray_ground_classifier_nodes_gtest
#     ${CLOUD_NODE_LIB}
#     ${OpenMP_LIBS}
#   )
#   ament_target_dependencies(test_ray_ground_classifier_nodes_gtest
#     "autoware_auto_common"
#     "rclcpp"
#     "ray_ground_classifier"
#     "sensor_msgs"
#   )

#   add_ros_test(
#     test/ray_ground_classifier_fuzzing.test.py
#     TIMEOUT "120"
#   )
#   add_ros_test(
#     test/ray_ground_classifier_node_launch.test.py
#     TIMEOUT "30"
#   )
# endif()

# ## install stuff
# ament_auto_package(INSTALL_TO_SHARE
#   param
#   launch)

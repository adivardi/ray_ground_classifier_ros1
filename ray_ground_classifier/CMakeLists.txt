# Copyright 2017-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# set(use_OMP FALSE)
# if(use_OMP)
#   find_package(OpenMP REQUIRED)
#   set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
#   set(OpenMP_LIBS gomp)
# else()
#   set(OpenMP_FLAGS "-Wno-unknown-pragmas")
# endif()

## dependencies
set(PROJECT_CATKIN_DEPS
    roscpp
    # pcl_conversions
    # pcl_ros
    # tf
    # visualization_msgs
    geometry_msgs
    # autoware_auto_common
    # autoware_auto_geometry
    # autoware_auto_algorithm
    autoware_auto_msgs
    sensor_msgs
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
  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# ament_auto_add_library(${PROJECT_NAME} SHARED
#   include/ray_ground_classifier/ray_aggregator.hpp
#   include/ray_ground_classifier/ray_ground_classifier.hpp
#   include/ray_ground_classifier/ray_ground_point_classifier.hpp
#   include/ray_ground_classifier/visibility_control.hpp
#   src/ray_aggregator.cpp
#   src/ray_ground_point_classifier.cpp
#   src/ray_ground_classifier.cpp
#   src/ray_ground_classifier_types.cpp)
# autoware_set_compile_options(${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_ray_ground_classifier_gtest
#     test/src/test_ray_ground_classifier.cpp
#   )
#   autoware_set_compile_options(test_ray_ground_classifier_gtest)
#   target_compile_options(test_ray_ground_classifier_gtest PRIVATE -Wno-conversion -Wno-double-promotion -Wno-float-conversion)
#   target_include_directories(test_ray_ground_classifier_gtest
#     PRIVATE "test/include" "include"
#   )
#   target_link_libraries(test_ray_ground_classifier_gtest
#     ${PROJECT_NAME}
#     ${OpenMP_LIBS}
#   )

#   ament_add_gtest(test_ray_ground_classifier_raytrace_gtest
#     test/src/test_ray_ground_classifier_raytrace.cpp
#   )
#   autoware_set_compile_options(test_ray_ground_classifier_raytrace_gtest)
#   target_compile_options(test_ray_ground_classifier_raytrace_gtest PRIVATE -Wno-conversion)
#   target_include_directories(test_ray_ground_classifier_raytrace_gtest
#     PRIVATE "test/include" "include"
#   )
#   target_link_libraries(test_ray_ground_classifier_raytrace_gtest
#     ${PROJECT_NAME}
#     ${OpenMP_LIBS}
#   )

#   ament_add_gtest(test_ray_aggregator_gtest
#     test/src/test_ray_aggregator.cpp
#   )
#   autoware_set_compile_options(test_ray_aggregator_gtest)
#   target_compile_options(test_ray_aggregator_gtest PRIVATE -Wno-float-conversion -Wno-conversion)

#   target_include_directories(test_ray_aggregator_gtest
#     PRIVATE "test/include" "include"
#   )
#   target_link_libraries(test_ray_aggregator_gtest
#     ${PROJECT_NAME}
#     ${OpenMP_LIBS}
#   )
#   if(use_OMP)
#     target_compile_options(test_ray_aggregator_gtest PRIVATE
#     ${OpenMP_FLAGS})
#   endif()
# endif()

# # workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
# target_compile_options(${PROJECT_NAME} PRIVATE
#     -Wno-sign-conversion
#     ${OpenMP_FLAGS}
# )


# # Ament Exporting
# ament_auto_package()

cmake_minimum_required(VERSION 3.8)
project(realsense_elevation_mapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

add_executable(local_elevation_mapper_node
  src/main.cpp
  src/local_elevation_mapper_node.cpp
  src/elevation_grid.cpp
  src/pointcloud_utils.cpp
)

target_include_directories(local_elevation_mapper_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(local_elevation_mapper_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(local_elevation_mapper_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
)

install(TARGETS local_elevation_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

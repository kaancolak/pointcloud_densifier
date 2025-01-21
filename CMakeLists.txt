cmake_minimum_required(VERSION 3.5)
project(pointcloud_densifier)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(pointcloud_densifier SHARED
  src/pointcloud_densifier_node.cpp)

target_link_libraries(pointcloud_densifier ${PCL_LIBRARIES})

rclcpp_components_register_node(pointcloud_densifier
  PLUGIN "pointcloud_densifier::PointCloudDensifierNode"
  EXECUTABLE pointcloud_densifier_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

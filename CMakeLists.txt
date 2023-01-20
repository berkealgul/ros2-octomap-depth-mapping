cmake_minimum_required(VERSION 3.5)
project(demaloc)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OpenMP REQUIRED)


include_directories(include)

add_library(octomap_demap SHARED
  src/demaloc.cpp)

ament_target_dependencies(octomap_demap
  rclcpp
  rclcpp_components
  message_filters
  cv_bridge
  octomap
  OpenCV
  octomap_msgs
  sensor_msgs
  geometry_msgs
  std_srvs)

ament_export_targets(octomap_demap HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS octomap_demap
  EXPORT octomap_demap
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

rclcpp_components_register_node(octomap_demap
  PLUGIN "octomap_depth_mapping::OctomapDemap"
  EXECUTABLE demaloc)

install(TARGETS demaloc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

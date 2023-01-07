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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(dataset_publisher src/dataset_publisher_node.cpp)
ament_target_dependencies(dataset_publisher
  rclcpp
  cv_bridge
  octomap
  OpenCV
  octomap_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(demaloc src/demaloc_node.cpp)
ament_target_dependencies(demaloc
  rclcpp
  cv_bridge
  octomap
  OpenCV
  octomap_msgs
  sensor_msgs
  geometry_msgs
)


install(TARGETS dataset_publisher demaloc
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

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


# user defined options
set(USE_CUDA TRUE)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(octomap REQUIRED)

include_directories(include)

# setup CUDA
if(USE_CUDA)
  find_package(CUDA REQUIRED)
  
  add_compile_definitions(CUDA)

  message("-- CUDA version: ${CUDA_VERSION}")
  
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_72,code=sm_72)

  add_library(octomap_demap SHARED
    src/demaloc.cpp
    src/cuda_proj.cu)

  set_property(TARGET octomap_demap PROPERTY CUDA_SEPARABLE_COMPILATION ON)

  ament_target_dependencies(octomap_demap CUDA)
else()
  add_library(octomap_demap SHARED
    src/demaloc.cpp)
endif()

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

rclcpp_components_register_node(octomap_demap
  PLUGIN "octomap_depth_mapping::OctomapDemap"
  EXECUTABLE demaloc)

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

install(TARGETS demaloc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

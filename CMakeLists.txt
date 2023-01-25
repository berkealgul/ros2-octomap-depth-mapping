cmake_minimum_required(VERSION 3.5)
project(octomap_depth_mapping)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# uncomment this line to use cuda
#set(USE_CUDA TRUE)
# You can change flags depending on your GPU
set(CUDA_GENCODE_FLAGS
  -gencode arch=compute_35,code=sm_35 
  -gencode arch=compute_37,code=sm_37 
  -gencode arch=compute_50,code=sm_50 
  -gencode arch=compute_52,code=sm_52 
  -gencode arch=compute_53,code=sm_53 
  -gencode arch=compute_60,code=sm_60 
  -gencode arch=compute_61,code=sm_61 
  -gencode arch=compute_62,code=sm_62 
  -gencode arch=compute_70,code=sm_70
  -gencode arch=compute_72,code=sm_72 
  -gencode arch=compute_75,code=sm_75 
  -gencode arch=compute_80,code=sm_80 
  -gencode arch=compute_86,code=sm_86 
  -gencode arch=compute_87,code=sm_87 
  -gencode arch=compute_87,code=compute_87)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

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
    ${CUDA_GENCODE_FLAGS})

  cuda_add_library(octomap_demap SHARED
    src/${PROJECT_NAME}.cpp
    src/cuda_proj.cu)
else()
  add_library(octomap_demap SHARED
    src/${PROJECT_NAME}.cpp)
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
  EXECUTABLE ${PROJECT_NAME})

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

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

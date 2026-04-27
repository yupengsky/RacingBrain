cmake_minimum_required(VERSION 3.8)
project(trt_cone_detector)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 75 86 87 89)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(test_cone_segmentation REQUIRED)

install(DIRECTORY models launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

find_package(CUDA QUIET)
find_library(NVINFER nvinfer)

if(NOT CUDA_FOUND OR NOT NVINFER)
  message(WARNING
    "CUDA/TensorRT was not found; trt_infer_node will not be built. "
    "Install CUDA and TensorRT on the target machine to enable PointPillars inference.")
  ament_package()
  return()
endif()

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
)

enable_language(CUDA)

add_library(trt_engine
  src/trt_detector.cpp
  src/voxel_generator.cu
  src/post_processor.cpp
)
target_link_libraries(trt_engine ${NVINFER} ${CUDA_LIBRARIES})
target_include_directories(trt_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(trt_infer_node src/cone_detector_node.cpp)
ament_target_dependencies(trt_infer_node
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  ament_index_cpp
  test_cone_segmentation
)
target_link_libraries(trt_infer_node trt_engine)

install(TARGETS
  trt_infer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  trt_engine
  DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_package()

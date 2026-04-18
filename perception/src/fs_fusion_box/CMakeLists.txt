cmake_minimum_required(VERSION 3.8)
project(fs_fusion_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(drd25_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cone_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

#3d的包
find_package(test_cone_segmentation REQUIRED)

# 2. 包含头文件
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# 3. 编译数学库 (改名为 fs_fusion_box_math)
add_library(fs_fusion_box_math 
  src/fs_fusion_box_math.cpp
)
ament_target_dependencies(fs_fusion_box_math
  rclcpp
  vision_msgs
  drd25_msgs
  cv_bridge
  OpenCV
  Eigen3
  test_cone_segmentation
)

# 4. 编译节点 (改名为 fusion_box_node)
add_executable(fusion_box_node 
  src/fs_fusion_box_node.cpp
)
# 注意：链接的库名也要改成新的 fs_fusion_box_math
target_link_libraries(fusion_box_node 
  fs_fusion_box_math
  ${OpenCV_LIBS}
)
ament_target_dependencies(fusion_box_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  message_filters
  drd25_msgs
  test_cone_segmentation
  cone_interfaces
)

# 5. 安装规则 (注意这里要安装新的名字)
install(TARGETS 
  fs_fusion_box_math
  fusion_box_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
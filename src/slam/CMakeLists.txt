cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 查找ros2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
# 查找自定义消息包
find_package(gnss_ins_msg REQUIRED)
find_package(drd25_msgs REQUIRED)
# 查找系统库 
find_package(Eigen3 REQUIRED)
find_library(PROJ_LIBRARY proj REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 定义可执行文件
add_executable(slam_node 
    src/slam_node.cpp
    src/loop_closure_detector.cpp
    )

# 头文件目录
target_include_directories(slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS})

# 链接ros依赖
ament_target_dependencies(slam_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  gnss_ins_msg
  drd25_msgs
)

# 链接系统库
target_link_libraries(slam_node
  ${PROJ_LIBRARY}
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

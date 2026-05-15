cmake_minimum_required(VERSION 3.20)
project(cpp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(drd25_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
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

add_executable(PID_controller_node
    src/PID_controller_node.cpp
    src/vehicle_pid_controller.cpp  
    src/longitudinal_vp.cpp
)

ament_target_dependencies(PID_controller_node 
  rclcpp 
  rcl_interfaces 
  nav_msgs
  fs_msgs 
  drd25_msgs
  tf2)

install(TARGETS
  PID_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.11)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gnss_ins_msg REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(talker src/publisher_member_function.cpp
               src/serial_stream.cpp)
ament_target_dependencies(talker Eigen3 rclcpp std_msgs gnss_ins_msg geometry_msgs tf2 tf2_geometry_msgs sensor_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(listener src/subscriber_member_function.cpp)

ament_target_dependencies(listener Eigen3 rclcpp std_msgs gnss_ins_msg geometry_msgs tf2 tf2_geometry_msgs sensor_msgs)

include_directories(
  include/cpp_pubsub/
  ${Eigen3_INCLUDE_DIRS}
)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

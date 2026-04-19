cmake_minimum_required(VERSION 3.8)
project(slam)

find_package(ament_cmake REQUIRED)

# TODO:
# 1. find_package(rclcpp REQUIRED)
# 2. find_package(gnss_ins_msg REQUIRED)
# 3. find_package(drd25_msgs REQUIRED)
# 4. 如果导出点云，补 PCL / sensor_msgs
# 5. add_executable(slam_node ...)
# 6. 安装 launch / config / rviz 资源

ament_package()

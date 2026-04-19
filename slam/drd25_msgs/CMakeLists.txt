cmake_minimum_required(VERSION 3.8)
project(drd25_msgs)

find_package(ament_cmake REQUIRED)

# TODO:
# 1. 打开 rosidl_default_generators
# 2. 注册 Cone / Map / Pointcloud / Path / Waypoint
# 3. 安装接口供 perception 与 slam 共用

ament_package()

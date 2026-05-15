cmake_minimum_required(VERSION 3.8)
project(test_cone_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. 查找依赖包
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# [新增] 消息生成必要的依赖
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(OpenMP_FOUND)
    message(STATUS "OpenMP found, enabling parallel processing.")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# ==========================================
# [新增] 2. 声明消息生成规则
# 必须在 add_executable 之前调用
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThreeDCone.msg"
  "msg/ThreeDConeArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

set(CSF_ROOT "" CACHE PATH "CSF install root containing include/CSF.h and lib/libCSF.*")
if(NOT CSF_ROOT AND DEFINED ENV{CSF_ROOT})
  set(CSF_ROOT "$ENV{CSF_ROOT}")
endif()

find_path(CSF_INCLUDE_DIR
  NAMES CSF.h
  HINTS
    "${CSF_ROOT}/include"
    "${CMAKE_SOURCE_DIR}/.ros_deps/csf/include"
)

find_library(CSF_LIBRARY
  NAMES CSF
  HINTS
    "${CSF_ROOT}/lib"
    "${CMAKE_SOURCE_DIR}/.ros_deps/csf/lib"
)

if(CSF_INCLUDE_DIR AND CSF_LIBRARY)
  set(HAVE_CSF 1)
  message(STATUS "CSF include: ${CSF_INCLUDE_DIR}")
  message(STATUS "CSF library: ${CSF_LIBRARY}")
else()
  set(HAVE_CSF 0)
  message(WARNING "CSF not found. Building cone segmentation with RANSAC-only ground removal.")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ==========================================
# 3. 编译可执行文件
# ==========================================
add_executable(cone_segmentation_node
  src/test_cone_segmentation.cpp
)

target_compile_definitions(cone_segmentation_node PRIVATE HAVE_CSF=${HAVE_CSF})
if(HAVE_CSF)
  target_include_directories(cone_segmentation_node PRIVATE ${CSF_INCLUDE_DIR})
endif()

ament_target_dependencies(cone_segmentation_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  builtin_interfaces
  rcl_interfaces
  nav_msgs
  message_filters
  geometry_msgs # [新增]
  std_msgs      # [新增]
)

target_link_libraries(cone_segmentation_node
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
if(HAVE_CSF)
  target_link_libraries(cone_segmentation_node ${CSF_LIBRARY})
endif()

# ==========================================
# [新增] 4. 确保节点能找到生成的消息头文件
# ==========================================
rosidl_target_interfaces(cone_segmentation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ==========================================
# 5. 安装规则
# ==========================================
install(TARGETS cone_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install( DIRECTORY config launch
         DESTINATION share/${PROJECT_NAME} 
)

# [新增] 安装 msg 文件，以便其他包可以引用
install(FILES
  msg/ThreeDCone.msg
  msg/ThreeDConeArray.msg
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()

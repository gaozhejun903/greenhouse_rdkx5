cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 自定义消息 ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PestDetection.msg"
  "msg/PestReport.msg"
)

# --- LiDAR driver ---
add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)

# --- Base driver ---
add_executable(base_driver src/base_driver.cpp)
ament_target_dependencies(base_driver rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

# --- Pest monitor (病虫害定位监控) ---
add_executable(pest_monitor src/pest_monitor.cpp)
ament_target_dependencies(pest_monitor
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
add_dependencies(pest_monitor ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(pest_monitor PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pest_monitor "${cpp_typesupport_target}")

# --- 安装节点 ---
install(TARGETS
  lidar_node
  base_driver
  pest_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本 (keyboard_control)
install(PROGRAMS
  scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

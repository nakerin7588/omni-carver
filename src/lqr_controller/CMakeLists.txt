cmake_minimum_required(VERSION 3.5)
project(lqr_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(iterative_lqr_controller SHARED
  src/iterative_lqr_controller.cpp
)

ament_target_dependencies(iterative_lqr_controller
  rclcpp_lifecycle
  tf2_ros
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav2_util
  geometry_msgs
  nav_msgs
  sensor_msgs
  Eigen3
)

# tell pluginlib where to find your XML
pluginlib_export_plugin_description_file(nav2_core iterative_lqr_controller_plugin.xml)

install(
  TARGETS iterative_lqr_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES iterative_lqr_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

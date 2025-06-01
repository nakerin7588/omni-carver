cmake_minimum_required(VERSION 3.5)
project(omni_sbmpo_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(omni_sbmpo_controller SHARED
  src/omni_sbmpo_controller.cpp
)

ament_target_dependencies(omni_sbmpo_controller
  rclcpp_lifecycle
  tf2_ros
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav2_util
  geometry_msgs
  nav_msgs
  Eigen3
)

# prevent pluginlib from falling back to Boost.Function
target_compile_definitions(omni_sbmpo_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# register the plugin with Nav2
pluginlib_export_plugin_description_file(nav2_core omni_sbmpo_controller_plugin.xml)

install(
  TARGETS omni_sbmpo_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES omni_sbmpo_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(omni_sbmpo_controller)
ament_export_dependencies(
  rclcpp_lifecycle
  tf2_ros
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav2_util
  geometry_msgs
  nav_msgs
  Eigen3
)

ament_package()

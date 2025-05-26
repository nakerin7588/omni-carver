cmake_minimum_required(VERSION 3.5)
project(hybrid_astar_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 (or higher)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include headers
include_directories(include)

# Name of the plugin library
set(library_name ${PROJECT_NAME}_plugin)

# Build the planner plugin library
add_library(${library_name} SHARED
  src/hybrid_astar_planner.cpp
)

# Link against Nav2 & ROS2 dependencies
ament_target_dependencies(${library_name}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  tf2_ros
  nav_msgs
  geometry_msgs
)

# Optional: disable boost functions for pluginlib
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export the plugin description so Nav2 can discover it
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# Install the plugin library
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install public headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the plugin XML description
install(
  FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Testing support (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export include dirs, libraries, and dependencies
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  tf2_ros
  nav_msgs
  geometry_msgs
)

ament_package()

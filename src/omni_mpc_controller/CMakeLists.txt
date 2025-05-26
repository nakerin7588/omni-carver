cmake_minimum_required(VERSION 3.5)
project(omni_mpc_controller)

# ========================================================
# 1. Find build‐time dependencies
# ========================================================
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# ========================================================
# 2. Nav2-specific macros and setup
# ========================================================
# This pulls in nav2-specific CMake functions, defines NAV2_PACKAGE(), etc.
nav2_package()

# ========================================================
# 3. Include paths and dependencies list
# ========================================================
include_directories(
  include
)

set(dependencies
  rclcpp_lifecycle
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

# ========================================================
# 4. Build the shared library
# ========================================================
add_library(${PROJECT_NAME} SHARED
  src/omni_mpc_controller.cpp
)

# Prevent pluginlib from falling back to Boost.Function
target_compile_definitions(${PROJECT_NAME} PUBLIC
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# ========================================================
# 5. Install targets and headers
# ========================================================
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ========================================================
# 6. Export plugin description for Nav2
# ========================================================
# This line makes sure omni_mpc_controller_plugin.xml is installed
# and picked up by pluginlib under the nav2_core namespace.
pluginlib_export_plugin_description_file(nav2_core
  omni_mpc_controller.xml
)

# ========================================================
# 7. Ament export for downstream packages
# ========================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

# ========================================================
# 8. Finalize
# ========================================================
ament_package()

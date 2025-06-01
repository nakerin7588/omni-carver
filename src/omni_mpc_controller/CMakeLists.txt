cmake_minimum_required(VERSION 3.5)
project(omni_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/omni_mpc_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# prevent pluginlib from falling back to Boost.Function
target_compile_definitions(omni_mpc_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES omni_mpc_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core omni_mpc_controller.xml)

ament_export_include_directories(include)
ament_export_libraries(omni_mpc_controller)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
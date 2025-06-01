cmake_minimum_required(VERSION 3.5)
project(omni_carver_pure_pursuit)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)

# nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  angles
)

add_library(omni_carver_pure_pursuit SHARED
        src/omni_carver_pure_pursuit.cpp)

# prevent pluginlib from using boost
target_compile_definitions(omni_carver_pure_pursuit PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(omni_carver_pure_pursuit
  ${dependencies}
)

install(TARGETS omni_carver_pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(omni_carver_pure_pursuit)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core omni_carver_pure_pursuit_plugin.xml)

ament_package()


cmake_minimum_required(VERSION 3.5)
project(omni_carver_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
######################################################

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  launch
  mesh
  rviz
  gazebo_description
  DESTINATION share/${PROJECT_NAME})

ament_package()

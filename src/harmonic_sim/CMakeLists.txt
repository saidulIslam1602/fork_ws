cmake_minimum_required(VERSION 3.5)
project(harmonic_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  meshes
  models
  rviz2_configs
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
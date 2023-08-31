cmake_minimum_required(VERSION 3.12)

project(motoman_resources)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

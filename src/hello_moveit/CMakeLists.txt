cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motoros2_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

find_package(control_msgs REQUIRED)

#simulation
add_executable(sim_hello_moveit src/sim_hello_moveit.cpp)
ament_target_dependencies(sim_hello_moveit motoros2_interfaces "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)


#server_traj_action.cpp
add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit motoros2_interfaces "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)

#move joints
add_executable(joint_hello_moveit src/joint_hello_moveit.cpp)
ament_target_dependencies(joint_hello_moveit motoros2_interfaces "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)


#add obstacle
add_executable(hello_obstacle src/hello_obstacle.cpp)
ament_target_dependencies(hello_obstacle motoros2_interfaces "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)

install(TARGETS
  sim_hello_moveit
  hello_moveit
  joint_hello_moveit
  hello_obstacle
  DESTINATION lib/${PROJECT_NAME})

ament_package()





cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motoros2_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

find_package(control_msgs REQUIRED)


# Add executable
add_executable(joint_state_subscriber src/joint_state_subscriber.cpp)
ament_target_dependencies(joint_state_subscriber rclcpp sensor_msgs)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


add_executable(enable_client src/enable_client.cpp)
ament_target_dependencies(enable_client rclcpp motoros2_interfaces std_srvs)


add_executable(disable_client src/disable_client.cpp)
ament_target_dependencies(disable_client rclcpp motoros2_interfaces std_srvs)

#client_traj_action.cpp
add_executable(client_traj src/client_traj_action.cpp)
ament_target_dependencies(client_traj rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs rclcpp motoros2_interfaces std_srvs)


#server_traj_action.cpp
add_executable(server_traj src/server_traj_action.cpp)
ament_target_dependencies(server_traj rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)

install(TARGETS
  joint_state_subscriber
  enable_client
  disable_client
  client_traj
  server_traj
  DESTINATION lib/${PROJECT_NAME})

ament_package()



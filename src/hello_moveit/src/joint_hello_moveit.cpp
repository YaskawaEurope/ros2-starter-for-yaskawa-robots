#include <memory>

#include "motoros2_interfaces/srv/start_traj_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  //-----------------------------------------------------
  rclcpp::init(argc, argv);

  //-------------------controller enable-----------------------------
  auto drive = rclcpp::Node::make_shared("enable_client");

  auto enable_client = drive->create_client<motoros2_interfaces::srv::StartTrajMode>("yaskawa/start_traj_mode");

  // Create a request message to send
  auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();

  // Send the request asynchronously
  auto future = enable_client->async_send_request(request);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  //-----------------------------------------------------

  auto const node = std::make_shared<rclcpp::Node>(
      "joint_hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  // using moveit::planning_interface::MoveGroupInterface;
  // auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Initialising and defining the planning group for move_base
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // A pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(move_group.getRobotModel()));

  // Set the joint names and values for the current state
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  std::vector<double> joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Set the joint positions in the current state
  current_state->setJointGroupPositions(move_group.getName(), joint_values);

  // Set the current state in the MoveGroupInterface
  move_group.setStartState(*current_state);

  // Get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(move_group.getName(), joint_group_positions);
  // Modifying one of the joint positions
  joint_group_positions[2] = 0.1; // radians
  joint_group_positions[3] = 0.5; // radians
  joint_group_positions[4] = 1.0; // radians
  // Pass the desired joint positions to move_group as goal for planning
  move_group.setJointValueTarget(joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //move_group.execute(my_plan);
  move_group.asyncExecute(my_plan);

  std::this_thread::sleep_for(std::chrono::seconds(10));

  rclcpp::shutdown();
  return 0;
}

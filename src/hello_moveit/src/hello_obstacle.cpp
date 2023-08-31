#include <memory>

#include "motoros2_interfaces/srv/start_traj_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Set a target Pose_1
  auto const target_pose1 = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.4;
    msg.position.y = 0.4;
    msg.position.z = 0.4;
    return msg;
  }();


    // Set a target Pose_1
  auto const target_pose2 = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.4;
    msg.position.y = -0.2;
    msg.position.z = 0.1;
    return msg;
  }();


  
  move_group_interface.setPoseTarget(target_pose1);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.1;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);


  // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]
  // {
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();


//   // Execute the plan
//   if (success)
//   {
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     RCLCPP_ERROR(logger, "Planning failed!");
//   }

//   RCLCPP_INFO(logger, "OK with robot movement. Take a next goal");

// //-----------------------------------------------------------------------
//   move_group_interface.setPoseTarget(target_pose2);

//     // Create a plan to that target pose
//   auto const [success2, plan2] = [&move_group_interface]
//   {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();


  // // Execute the plan
  // if (success2)
  // {
  //   move_group_interface.execute(plan2);
  // }
  // else
  // {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }


  // Shutdown ROS
 // rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <memory>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "motoros2_interfaces/srv/start_traj_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("action_client")
    {

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "manipulator/follow_joint_trajectory");

        auto node = rclcpp::Node::make_shared("enable_client");

        auto enable_client = node->create_client<motoros2_interfaces::srv::StartTrajMode>("manipulator/start_traj_mode");

        // Create a request message to send
        auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();

        // Send the request asynchronously
        auto future = enable_client->async_send_request(request);

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    // private:
    std::time_t getCurrentTimeAsString()
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
        return currentTime;
    }
    void moveRobot()
    {
        RCLCPP_INFO(get_logger(), "MOVE ROBOT.");

        auto goal = std::make_shared<FollowJointTrajectory::Goal>();
        goal->trajectory = JointTrajectory();
        // auto goal = FollowJointTrajectory::Goal();
        // goal.trajectory = JointTrajectory();

        goal->trajectory.joint_names = {"manipulator/joint_1", "manipulator/joint_2", "manipulator/joint_3",
                                        "manipulator/joint_4", "manipulator/joint_5", "manipulator/joint_6"};
        goal->trajectory.points.resize(2);
        goal->trajectory.points[0].time_from_start = rclcpp::Duration(2, 0);
        goal->trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        goal->trajectory.points[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        goal->trajectory.points[0].accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        goal->trajectory.points[0].effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        goal->trajectory.points[1].time_from_start = rclcpp::Duration(6, 0);
        goal->trajectory.points[1].positions = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
        goal->trajectory.points[1].velocities = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
        goal->trajectory.points[1].accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        goal->trajectory.points[1].effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        goal->trajectory.header.stamp = rclcpp::Clock().now();
        
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
   

        auto goal_handle_future = action_client_->async_send_goal(*goal);

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
            return;
        }
    }

    void goalResponseCallback(std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by the action server.");
        }
    }

    void feedbackCallback(
        GoalHandleFollowJointTrajectory::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        // Process feedback if needed
    }

    void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded. Robot joints moved successfully.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal aborted by the action server.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal canceled by the action client.");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
            break;
        }
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create an instance of the FollowJointTrajectoryClient class
    auto client = std::make_shared<RobotController>();

    // Send the action goal
    client->moveRobot();

    // Spin the node
    rclcpp::spin(client);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}



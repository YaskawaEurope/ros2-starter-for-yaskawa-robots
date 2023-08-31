#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class RobotControllerServer : public rclcpp::Node
{
public:
    RobotControllerServer() : Node("robot_controller_server")
    {
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "robot_joint_trajectory",
            std::bind(&RobotControllerServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotControllerServer::handleCancel, this, std::placeholders::_1),
            std::bind(&RobotControllerServer::handleAccepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order ");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received cancel request for goal with UUID: ");
        // Process the cancel request and stop executing the trajectory

        // Return true to successfully handle the cancel request
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Goal with UUID %s has been accepted by the action server.");
        // Start executing the trajectory
    }

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

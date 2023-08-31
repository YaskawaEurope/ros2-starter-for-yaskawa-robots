#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using JointState = sensor_msgs::msg::JointState;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using SetTrajectoryMode = motoros2_interfaces::srv::StartTrajMode;



//--------------------------------------
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJointTrajectoryClient = rclcpp_action::Client<FollowJointTrajectory>;
class SimpleTrajectoryActionClient : public rclcpp::Node
{
private:
    FollowJointTrajectoryClient::SharedPtr action_client_;

public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using FollowJointTrajectoryClient = rclcpp_action::Client<FollowJointTrajectory>;

    SimpleTrajectoryActionClient() : Node("my_node")
    {

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    // Create an action client with the QoS profile
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        shared_from_this(), "follow_joint_trajectory", qos_profile);


        // Create a QoS profile with reliability policy
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        // Create node options with the desired QoS profile
        rclcpp::NodeOptions options;
        //options.qos_overrides(qos_profile);

        // Create an action client with QoS
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/follow_joint_trajectory", qos_profile);

        call_start_traj_mode_service();

        // Wait for the action server to become available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
        }
    }

    //---------------------------------

    void listener_callback(const JointState::SharedPtr msg)
    {
        // Update joint state information if needed
    }

    void call_start_traj_mode_service()
    {
        auto client = create_client<SetTrajectoryMode>("/start_traj_mode");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Waiting for service");
        }

        auto request = std::make_shared<SetTrajectoryMode::Request>();

        auto future = client->async_send_request(request);
        future.wait();

        if (future.valid() && future.get()->success)
        {
            RCLCPP_INFO(get_logger(), "start_traj_mode service call succeeded!");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "start_traj_mode service call failed");
        }
    }

    void return_joint_state()
    {
        // Implement joint state retrieval if needed
    }

    void send_goal()
    {
        auto goal_msg = std::make_shared<FollowJointTrajectory::Goal>();
        goal_msg->trajectory = JointTrajectory();

        goal_msg->goal_time_tolerance = rclcpp::Duration(0, 0).to_msg();

        goal_msg->trajectory.joint_names = {
            "group_1/joint_1",
            "group_1/joint_2",
            "group_1/joint_3",
            "group_1/joint_4",
            "group_1/joint_5",
            "group_1/joint_6"};

        std::vector<double> q0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> q1 = q0;
        std::vector<double> q2 = q0;
        std::vector<double> q3 = q0;

        q1[5] -= M_PI / 9.0;
        q1[0] -= M_PI / 18.0;

        q2[0] += M_PI / 9.0;
        q2[1] += M_PI / 18.0;
        q2[2] += M_PI / 18.0;

        q3[0] -= M_PI / 18.0;
        q3[1] -= M_PI / 18.0;
        q3[2] -= M_PI / 18.0;

        // Make the robot come to a complete stop at each trajectory point (i.e., zero target velocity).
        std::vector<double> qdot(goal_msg->trajectory.joint_names.size(), 0.0);

        goal_msg->trajectory.header.stamp = this->now();

        // Add trajectory points
        goal_msg->trajectory.points.push_back(JointTrajectoryPoint());
        goal_msg->trajectory.points[0].positions = q0;
        goal_msg->trajectory.points[0].velocities = qdot;
        goal_msg->trajectory.points[0].accelerations = qdot;
        goal_msg->trajectory.points[0].effort = qdot;
        goal_msg->trajectory.points[0].time_from_start = rclcpp::Duration(0, 0).to_msg();

        goal_msg->trajectory.points.push_back(JointTrajectoryPoint());
        goal_msg->trajectory.points[1].positions = q1;
        goal_msg->trajectory.points[1].velocities = qdot;
        goal_msg->trajectory.points[1].accelerations = qdot;
        goal_msg->trajectory.points[1].effort = qdot;
        goal_msg->trajectory.points[1].time_from_start = rclcpp::Duration(5, 0).to_msg();

        goal_msg->trajectory.points.push_back(JointTrajectoryPoint());
        goal_msg->trajectory.points[2].positions = q2;
        goal_msg->trajectory.points[2].velocities = qdot;
        goal_msg->trajectory.points[2].accelerations = qdot;
        goal_msg->trajectory.points[2].effort = qdot;
        goal_msg->trajectory.points[2].time_from_start = rclcpp::Duration(10, 0).to_msg();

        goal_msg->trajectory.points.push_back(JointTrajectoryPoint());
        goal_msg->trajectory.points[3].positions = q3;
        goal_msg->trajectory.points[3].velocities = qdot;
        goal_msg->trajectory.points[3].accelerations = qdot;
        goal_msg->trajectory.points[3].effort = qdot;
        goal_msg->trajectory.points[3].time_from_start = rclcpp::Duration(15, 0).to_msg();

        RCLCPP_INFO(get_logger(), "Waiting for driver's action server to become available ...");
        action_client_->wait_for_action_server();
        RCLCPP_INFO(get_logger(), "Connected to trajectory action server");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&SimpleTrajectoryActionClient::feedback_callback, this, std::placeholders::_1);
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

        if (rclcpp::spin_until_future_complete(get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send goal");
            return;
        }

        auto goal_handle = goal_handle_future.get();

        if (!goal_handle.accepted)
        {
            RCLCPP_INFO(get_logger(), "Goal rejected");
            return;
        }

        RCLCPP_INFO(get_logger(), "Goal accepted");

        auto result_future = action_client_->async_get_result(goal_handle);

        if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get result");
            return;
        }

        auto result = result_future.get();

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(get_logger(), "Goal execution failed");
            return;
        }

        RCLCPP_INFO(get_logger(), "Goal execution succeeded");

        // Additional code after goal execution succeeds

        // Cleanup and shutdown
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTrajectoryActionClient>();

    // Call the necessary functions
    node->call_start_traj_mode_service();
    node->send_goal();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

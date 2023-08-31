#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
        : Node("joint_state_subscriber")
    {
        //      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //         "/joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS(),
            std::bind(&JointStateSubscriber::topic_callback, this, std::placeholders::_1));

        // subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states",
        //     rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        //     std::bind(&JointStateSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        std::vector<double> joint_positions;
        for (size_t i = 0; i < msg->name.size(); i++)
        {
         //   if (msg->name[i].substr(0, 6) == "group_1/joint_")
          //  { // assuming joint names start with "joint_"
                joint_positions.push_back(msg->position[i]);
          //  }
        }
        RCLCPP_INFO(this->get_logger(), "Joint positions: [%f, %f, %f, %f, %f, %f]",
                    joint_positions[0], joint_positions[1], joint_positions[2],
                    joint_positions[3], joint_positions[4], joint_positions[5]);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

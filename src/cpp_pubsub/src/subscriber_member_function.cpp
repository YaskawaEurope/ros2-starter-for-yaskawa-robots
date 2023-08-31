#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    // "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    subscription_ = this->create_subscription<std_msgs::msg::String>("topic",
                                                                     rclcpp::SystemDefaultsQoS(),
                                                                     [&](const std_msgs::msg::String::SharedPtr msg)
                                                                     {
                                                                       topic_callback(msg);
                                                                     });

    check();
  }

  void check()
  {

    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    RCLCPP_INFO(this->get_logger(), "CHECKING:");
    // std::this_thread::sleep_for(std::chrono::seconds(1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

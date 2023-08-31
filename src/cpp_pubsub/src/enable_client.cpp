#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motoros2_interfaces/srv/start_traj_mode.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a node
  auto node = rclcpp::Node::make_shared("enable_client");

  // Create a service client to send the StartTrajMode service request
  auto enable_client = node->create_client<motoros2_interfaces::srv::StartTrajMode>("yaskawa/start_traj_mode");

  // Create a request message to send
  auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();

  // Send the request asynchronously
  auto future = enable_client->async_send_request(request);

  std::this_thread::sleep_for(std::chrono::seconds(5));
  // Wait for the future to complete and print the response
  //future.wait();

  // if (future.valid() && future.get()-> success) {
  //   RCLCPP_INFO(node->get_logger(), "StartTrajMode service request was successful");
  // } else {
  //   RCLCPP_ERROR(node->get_logger(), "StartTrajMode service request failed");
  // }

  rclcpp::shutdown();
  return 0;
}

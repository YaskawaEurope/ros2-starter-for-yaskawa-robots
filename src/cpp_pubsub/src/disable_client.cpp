#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a node
  auto node = rclcpp::Node::make_shared("disable_client");

  // Create a service client to send the StartTrajMode service request
  auto enable_client = node->create_client<std_srvs::srv::Trigger>("yaskawa/stop_traj_mode");

  // Create a request message to send
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the request asynchronously
  auto future = enable_client->async_send_request(request);

  std::this_thread::sleep_for(std::chrono::seconds(5));
  //future.wait();


  rclcpp::shutdown();
  return 0;
}




// class TrajModeStopper : public rclcpp::Node
// {
// public:
//     TrajModeStopper() : Node("traj_mode_stopper")
//     {
//         auto stop_client_ = this->create_client<std_srvs::srv::Trigger>("stop_traj_mode");
//         while (!stop_client_->wait_for_service(std::chrono::seconds(1))) {
//             if (!rclcpp::ok()) {
//                 RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//                 return;
//             }
//             RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
//         }

//         stop_robot();
//     }

// private:
//     void stop_robot()
//     {
//         auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//         auto future = stop_client_->async_send_request(request);
//         future.wait();

//         std_srvs::srv::Trigger::Response::SharedPtr response = future.get();

//         // Console log of response
//         //RCLCPP_INFO(this->get_logger(), "Response Result Code Value: %u", response->result_code.value);

//         // Check
//         if (response){
//         RCLCPP_INFO(this->get_logger(), "Trajectory mode stopped");
//         }
//     }

//     rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TrajModeStopper>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

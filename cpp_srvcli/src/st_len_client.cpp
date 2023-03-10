#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/st_len.hpp"                                       // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: st_len_client X Y ");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("st_len_client");  // CHANGE
  rclcpp::Client<tutorial_interfaces::srv::StLen>::SharedPtr client =                // CHANGE
    node->create_client<tutorial_interfaces::srv::StLen>("st_len");          // CHANGE

  auto request = std::make_shared<tutorial_interfaces::srv::StLen::Request>();       // CHANGE
 // request->a = atoll(argv[1]);
  request->a = (argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}

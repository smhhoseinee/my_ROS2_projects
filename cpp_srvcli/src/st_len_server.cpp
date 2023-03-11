#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/st_len.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::StLen::Request> request,
         std::shared_ptr<tutorial_interfaces::srv::StLen::Response> response,
         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
  int sum = 0;
  for (const auto& c : request->a) {
    sum += static_cast<int>(c);
  }
  response->sum = sum;

  std_msgs::msg::String msg;
  msg.data = "st_len_server said: " + request->a + ", " + std::to_string(request->a.length()) + ", " + std::to_string(sum);
  publisher->publish(msg);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s b: %ld",
              request->a.c_str(), request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("st_len_server");
  rclcpp::Service<tutorial_interfaces::srv::StLen>::SharedPtr service =
      node->create_service<tutorial_interfaces::srv::StLen>("st_len",  std::bind(&add, std::placeholders::_1, std::placeholders::_2, node->create_publisher<std_msgs::msg::String>("/conversation", 10)));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get st len.");
  rclcpp::spin(node);
  rclcpp::shutdown();
}

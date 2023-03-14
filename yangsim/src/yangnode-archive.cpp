#include "rclcpp/rclcpp.hpp"
#include "yinyang_msgs/srv/st_len.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

char* msgs[] = {
  "Hi Yin, I am Yang the opposite of you.",
  "Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation",
  "Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.",
  "For what is and what is not beget each other.",
  "High and low place each other.",
  "Before and behind follow each other.",
  "And you fade into the darkness."
};


void add(const std::shared_ptr<yinyang_msgs::srv::StLen::Request> request,
         std::shared_ptr<yinyang_msgs::srv::StLen::Response> response,
         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
  int sum = 0;
  for (const auto& c : request->a) {
    sum += static_cast<int>(c);
  }
  response->sum = sum;

  std_msgs::msg::String msg;
  msg.data = "yinnode said: " + request->a + ", " + std::to_string(request->a.length()) + ", " + std::to_string(sum);
  publisher->publish(msg);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s b: %ld",
              request->a.c_str(), request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("yangnode");

  rclcpp::Service<yinyang_msgs::srv::StLen>::SharedPtr service =
      node->create_service<yinyang_msgs::srv::StLen>("yangnode_service", std::bind(&add, std::placeholders::_1, std::placeholders::_2, node->create_publisher<std_msgs::msg::String>("/conversation", 10)));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get st len.");

  rclcpp::Client<yinyang_msgs::srv::StLen>::SharedPtr client =                // CHANGE
    node->create_client<yinyang_msgs::srv::StLen>("yinnode_service");          // CHANGE

  std::string pm(str[this->count]);

  auto request = std::make_shared<yinyang_msgs::srv::StLen::Request>();       // CHANGE
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }


  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

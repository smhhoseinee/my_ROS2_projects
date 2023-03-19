#include "rclcpp/rclcpp.hpp"
#include "yinyang_msgs/srv/st_len.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


#include <chrono>
#include <memory>
#include <sstream>
#include <future>
#include <string>
#include <functional>

using namespace std::chrono_literals;

char* str[] = {
  "Hi Yin, I am Yang the opposite of you.",
  "Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation",
  "Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.",
  "For what is and what is not beget each other.",
  "High and low place each other.",
  "Before and behind follow each other.",
  "And you fade into the darkness."
};

int count = 0;
bool shout;

void yang_service(const std::shared_ptr<yinyang_msgs::srv::StLen::Request> request,
         std::shared_ptr<yinyang_msgs::srv::StLen::Response> response,
         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
  int sum = 0;
  for (const auto& c : request->a) {
    sum += static_cast<int>(c);
  }
  response->sum = sum;

  std_msgs::msg::String msg;
  msg.data = "Yin said: " + request->a + ", " + std::to_string(request->a.length()) + ", " + std::to_string(sum);

  publisher->publish(msg);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s b: %ld",
              request->a.c_str(), request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
}


void send_request(rclcpp::Client<yinyang_msgs::srv::StLen>::SharedPtr client, std::shared_ptr<rclcpp::Node> node, const char* msg)
{

  
  std::string pm(str[count]);
  if (shout) {
    pm = "**" + pm + "**";
  }
  auto request = std::make_shared<yinyang_msgs::srv::StLen::Request>();
  request->a = pm;
  request->b = std::strlen(str[count]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  // count = (count + 1) % (sizeof(str) / sizeof(str[0]));
  // count = count + 1;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("yangnode");

  // Create a parameter for shouting messages
  node->declare_parameter<bool>("shout", true);
  
  // Get the parameter value, if it exists
  shout = node->get_parameter("shout").get_parameter_value().get<bool>();

  // create a service
  rclcpp::Service<yinyang_msgs::srv::StLen>::SharedPtr service =
      node->create_service<yinyang_msgs::srv::StLen>("yangnode_service", std::bind(&yang_service, std::placeholders::_1, std::placeholders::_2, node->create_publisher<std_msgs::msg::String>("/conversation", 10)));
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get st len.");

  // create a client
  rclcpp::Client<yinyang_msgs::srv::StLen>::SharedPtr client =                // CHANGE
    node->create_client<yinyang_msgs::srv::StLen>("yinnode_service");          // CHANGE

  // Send str array messages one by one to yinnode_server
  while (count < sizeof(str)/sizeof(*str)) {
    send_request(client, node, str[count]);
    count++;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "yangsim/srv/strlen.hpp"
#include <string>

void strlen_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<yangsim::srv::Strlen_Request> request,
  std::shared_ptr<yangsim::srv::Strlen_Response> response)
{
  (void)request_header;
  std::string str = request->str;
  response->length = str.length();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: %s, sending back response: %ld", str.c_str(), response->length);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("strlen_server");
  auto service = node->create_service<yangsim::srv::Strlen>("strlen", &strlen_callback);
  RCLCPP_INFO(node->get_logger(), "Ready to compute string length.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


//#include "rclcpp/rclcpp.hpp"
//#include "yangsim/srv/strlen.hpp"
//
//#include <memory>
//#include <string>
//
//void strlen_callback(const std::shared_ptr<yangsim::srv::Strlen::Request> request,
//          std::shared_ptr<yangsim::srv::Strlen::Response> response)
//{
//  std::string str = request->input_string;
//  response->length = str.length();
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nInput string: %s", str.c_str());
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", response->length);
//}
//
//int main(int argc, char **argv)
//{
//  rclcpp::init(argc, argv);
//
//  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("strlen_server");
//
//  rclcpp::Service<yangsim::srv::Strlen>::SharedPtr service =
//    node->create_service<yangsim::srv::Strlen>("strlen", &strlen_callback);
//
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to find length of a string.");
//
//  rclcpp::spin(node);
//  rclcpp::shutdown();
//}
//

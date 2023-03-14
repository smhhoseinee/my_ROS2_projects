#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#define MAX 7

using namespace std::chrono_literals;

namespace cb_group_demo
{
class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("client_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("conversation", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&DemoNode::timer_callback, this));
    }

private:
  int count_ = 0;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    std::stringstream msg;
    msg << "Message " << count_ << ": Hello from Yang!";
    auto message = std_msgs::msg::String();
    message.data = msg.str();

    publisher_->publish(message);

    if (++count_ == MAX) {
      RCLCPP_INFO(this->get_logger(), "Reached maximum number of messages. Exiting...");
      rclcpp::shutdown();
    }
  }
};
}   // namespace cb_group_demo

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cb_group_demo::DemoNode>();
    rclcpp::spin(client_node);
    rclcpp::shutdown();

    return 0;
}
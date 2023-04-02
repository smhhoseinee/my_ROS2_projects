#include "my_package/my_component.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace my_package
{
  MyComponent::MyComponent(const rclcpp::NodeOptions & options)
    : Node("my_node", options), count_(0)
  {
    // Create a publisher of "std_msgs/String" messages on the "chatter" topic.
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(1s, std::bind(&MyComponent::on_timer, this));
  }

  void MyComponent::on_timer()
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());

    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg));
  }
}  // namespace my_package


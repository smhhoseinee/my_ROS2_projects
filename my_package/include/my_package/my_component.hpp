#ifndef MY_PACKAGE__MY_COMPONENT_HPP_
#define MY_PACKAGE__MY_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_package
{
  class MyComponent : public rclcpp::Node
  {
  public:
    explicit MyComponent(const rclcpp::NodeOptions & options);

  private:
    void on_timer();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
  };
}  // namespace my_package

#endif  // MY_PACKAGE__MY_COMPONENT_HPP_

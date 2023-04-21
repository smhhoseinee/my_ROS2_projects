#include <memory>

#include "my_talker/talker_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto talker = std::make_shared<my_talker::Talker>(options);
  exec.add_node(talker);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}




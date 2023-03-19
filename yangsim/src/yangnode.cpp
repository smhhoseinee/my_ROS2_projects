#include <chrono>
#include <memory>
#include <sstream>
#include <future>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// #include <cb_group_demo/demo_node.hpp>

#include "yinyang_msgs/srv/st_len.hpp"
#include "yinyang_msgs/action/farewell.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using Farewell = yinyang_msgs::action::Farewell;
using GoalHandle = rclcpp_action::ClientGoalHandle<Farewell>;
    const int Messages_Limit = 7;
    char* str[] = {
      "Hi Yin, I am Yang the opposite of you.",
      "Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation",
      "Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.",
      "For what is and what is not beget each other.",
      "High and low place each other.",
      "Before and behind follow each other.",
      "And you fade into the darkness."
    };

namespace cb_group_demo
{
class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("client_node")
    {
        client_cb_group_ = nullptr;
        timer_cb_group_ = nullptr;

        this->declare_parameter("shout", false);

        publisher_ = this->create_publisher<std_msgs::msg::String>("conversation", 10);

        client_ptr_ = this->create_client<yinyang_msgs::srv::StLen>(
          "yinnode_service", 
          rmw_qos_profile_services_default, client_cb_group_);
        
        service_ptr_ = this->create_service<yinyang_msgs::srv::StLen>(
          "yangnode_service",
          std::bind(&DemoNode::service_callback, this, _1, _2, _3)
        );
        timer_ptr_ = this->create_wall_timer(0.5s, std::bind(&DemoNode::timer_callback, this),
                                            timer_cb_group_);
                            
        action_client_ptr_ = rclcpp_action::create_client<Farewell>(
          this,
          "farewell");
        
    }

private:

    int count = 0;
    // bool time_to_send = false;
    bool time_to_send = false;
    bool goal_sent = false;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Client<yinyang_msgs::srv::StLen>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    rclcpp::Service<yinyang_msgs::srv::StLen>::SharedPtr service_ptr_;

    rclcpp_action::Client<Farewell>::SharedPtr action_client_ptr_;


    void goal_response_callback(const GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const Farewell::Feedback> feedback) {
      
      std::stringstream feedback_str;
      auto number = feedback->opacity;
      feedback_str << number << " ";
      RCLCPP_INFO(this->get_logger(), feedback_str.str().c_str());
    }

    void result_callback(const GoalHandle::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      std::stringstream result_str;
      result_str << "Result received: ";
      result_str << result.result->res;
      RCLCPP_INFO(this->get_logger(), result_str.str().c_str());

      throw 0;
    }



    // void service_callback(const std::shared_ptr<yinyang_msgs::srv::StLen::Request> request,
    //      std::shared_ptr<yinyang_msgs::srv::StLen::Response> response,
    //      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
    // {  

    void service_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<yinyang_msgs::srv::StLen::Request> request,
            const std::shared_ptr<yinyang_msgs::srv::StLen::Response> response)
    {

        int sum = 0;
        for (const auto& c : request->a) {
          sum += static_cast<int>(c);
        }
        response->sum = sum;

        std_msgs::msg::String msg;
        msg.data = "Yin said: " + request->a + ", " + std::to_string(request->a.length()) + ", " + std::to_string(sum);
        publisher_->publish(msg);
        this->time_to_send = true;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s b: %ld",
                    request->a.c_str(), request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", (long int)response->sum);
    }



    void timer_callback()
    {
        
        if (count >= Messages_Limit && !goal_sent) {
            auto goal_msg = Farewell::Goal();
            goal_msg.req = "Good bye";
            RCLCPP_INFO(get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<Farewell>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&DemoNode::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&DemoNode::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&DemoNode::result_callback, this, _1);
            action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

            goal_sent = true;
        }
        else if (time_to_send && count < Messages_Limit) {
            bool shout = get_parameter("shout").get_parameter_value().get<bool>();
            auto request = std::make_shared<yinyang_msgs::srv::StLen::Request>();

            const std::string& pm = str[count];
            request->a = pm;
            request->b = pm.length();

            if (shout) {
                request->a = "**" + pm + "**";
            }

            ++count;
            auto result_future = client_ptr_->async_send_request(request);
            time_to_send = false;
            RCLCPP_INFO(get_logger(), "request sent");
        }
        
    }

}; 
} 

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<cb_group_demo::DemoNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);

    RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

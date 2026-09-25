#include "steelhead_example/component_one.hpp"
using std::placeholders::_1;

namespace steelhead_example
{


ComponentOne::ComponentOne(const rclcpp::NodeOptions & options)
: Node("component_one", options) 
{
    this->declare_parameter<int>("example_param", 0);
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("example/component_one/out", rclcpp::QoS(10));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "example/component_one/in", 10, std::bind(&ComponentOne::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Component One succesfully started!");
}


void ComponentOne::callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "In Component One");
    auto message = std_msgs::msg::String();
    message.data = msg->data + " from ComponentOne";
    publisher_->publish(message);
}

    
} // namespace steelhead_example

int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions();
        rclcpp::spin(std::make_shared<steelhead_example::ComponentOne>(options));
        rclcpp::shutdown();
    }
    catch (rclcpp::exceptions::RCLError const &)
    {
        // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
    } // during testing sometimes throws error
    return 0;
}
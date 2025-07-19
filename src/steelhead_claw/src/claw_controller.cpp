#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

class ClawController : public rclcpp::Node
{
public:
    ClawController() : Node("claw_controller")
    {
        // Publisher for position commands
        position_command_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/steelhead/claw/position_command", 10);
        
        // Services for open/close operations
        open_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/steelhead/claw/open",
            std::bind(&ClawController::open_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        close_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/steelhead/claw/close",
            std::bind(&ClawController::close_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Subscribe to position feedback
        position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/steelhead/claw/position", 10,
            std::bind(&ClawController::position_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Claw controller initialized");
    }

private:
    void open_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            send_position_command(1.0); // Fully open
            response->success = true;
            response->message = "Claw opening command sent";
        } else {
            response->success = false;
            response->message = "Invalid request data";
        }
    }
    
    void close_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            send_position_command(0.0); // Fully closed
            response->success = true;
            response->message = "Claw closing command sent";
        } else {
            response->success = false;
            response->message = "Invalid request data";
        }
    }
    
    void position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_position_ = msg->data;
    }
    
    void send_position_command(double position)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = position;
        position_command_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sending claw position command: %.2f", position);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_command_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr open_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr close_service_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    
    double current_position_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClawController>());
    rclcpp::shutdown();
    return 0;
}
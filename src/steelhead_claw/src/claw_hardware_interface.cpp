#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>

class ClawHardwareInterface : public rclcpp::Node
{
public:
    ClawHardwareInterface() : Node("claw_hardware_interface")
    {
        // Subscribe to claw commands
        claw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/steelhead/claw/position_command", 10,
            std::bind(&ClawHardwareInterface::claw_command_callback, this, std::placeholders::_1));
        
        // Publisher for claw status
        claw_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steelhead/claw/position", 10);
        
        // Timer for publishing status
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ClawHardwareInterface::publish_status, this));
            
        RCLCPP_INFO(this->get_logger(), "Claw hardware interface initialized");
    }

private:
    void claw_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Clamp position between 0.0 (closed) and 1.0 (open)
        target_position_ = std::max(0.0, std::min(1.0, msg->data));
        
        // Send command to actual hardware (servo/actuator)
        // This is where you'd interface with your physical claw controller
        send_hardware_command(target_position_);
        
        RCLCPP_INFO(this->get_logger(), "Claw position command: %.2f", target_position_);
    }
    
    void send_hardware_command(double position)
    {
        // TODO: Implement actual hardware communication
        // Examples:
        // - Serial communication to servo controller
        // - PWM signal generation
        // - I2C/SPI communication
        
        // For now, simulate instant response
        current_position_ = position;
        
        // In real implementation, you might do something like:
        // servo_controller_.set_position(position * servo_range_);
        // or
        // pwm_output_.write(position_to_pwm(position));
    }
    
    void publish_status()
    {
        auto status_msg = std_msgs::msg::Float64();
        status_msg.data = current_position_;
        claw_status_pub_->publish(status_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr claw_command_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr claw_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double target_position_ = 0.0;
    double current_position_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClawHardwareInterface>());
    rclcpp::shutdown();
    return 0;
}
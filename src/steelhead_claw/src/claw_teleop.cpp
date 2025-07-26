#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class ClawTeleop : public rclcpp::Node
{
public:
    ClawTeleop() : Node("claw_teleop")
    {
        // Service clients for claw control
        open_client_ = this->create_client<std_srvs::srv::SetBool>("/steelhead/claw/open");
        close_client_ = this->create_client<std_srvs::srv::SetBool>("/steelhead/claw/close");
        
        // Position command publisher for fine control
        position_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steelhead/claw/position_command", 10);
        
        // Position feedback subscriber
        position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/steelhead/claw/position", 10,
            std::bind(&ClawTeleop::position_callback, this, std::placeholders::_1));
        
        // Setup non-blocking keyboard input
        setup_keyboard();
        
        // Timer for checking keyboard input
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ClawTeleop::check_keyboard, this));
        
        print_instructions();
        
        RCLCPP_INFO(this->get_logger(), "Claw teleop node started");
    }
    
    ~ClawTeleop()
    {
        restore_keyboard();
    }

private:
    void setup_keyboard()
    {
        // Get current terminal attributes
        tcgetattr(STDIN_FILENO, &original_termios_);
        
        // Set new attributes for non-blocking input
        struct termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        
        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    
    void restore_keyboard()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }
    
    void print_instructions()
    {
        RCLCPP_INFO(this->get_logger(), "\n=== Claw Teleop Controls ===");
        RCLCPP_INFO(this->get_logger(), "o : Open claw");
        RCLCPP_INFO(this->get_logger(), "c : Close claw");
        RCLCPP_INFO(this->get_logger(), "+ : Increase claw opening");
        RCLCPP_INFO(this->get_logger(), "- : Decrease claw opening");
        RCLCPP_INFO(this->get_logger(), "s : Show current position");
        RCLCPP_INFO(this->get_logger(), "q : Quit");
        RCLCPP_INFO(this->get_logger(), "==========================\n");
    }
    
    void check_keyboard()
    {
        char key;
        if (read(STDIN_FILENO, &key, 1) > 0) {
            handle_key(key);
        }
    }
    
    void handle_key(char key)
    {
        switch (key) {
            case 'o':
            case 'O':
                call_open_service();
                break;
            case 'c':
            case 'C':
                call_close_service();
                break;
            case '+':
            case '=':
                adjust_position(0.1);
                break;
            case '-':
            case '_':
                adjust_position(-0.1);
                break;
            case 's':
            case 'S':
                show_position();
                break;
            case 'q':
            case 'Q':
                RCLCPP_INFO(this->get_logger(), "Quitting claw teleop...");
                rclcpp::shutdown();
                break;
            default:
                // Ignore other keys
                break;
        }
    }
    
    void call_open_service()
    {
        if (!open_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Open service not available");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        
        auto future = open_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Opening claw...");
    }
    
    void call_close_service()
    {
        if (!close_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Close service not available");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        
        auto future = close_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Closing claw...");
    }
    
    void adjust_position(double delta)
    {
        current_position_ = std::max(0.0, std::min(1.0, current_position_ + delta));
        
        auto msg = std_msgs::msg::Float64();
        msg.data = current_position_;
        position_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Adjusting claw position to: %.2f", current_position_);
    }
    
    void show_position()
    {
        RCLCPP_INFO(this->get_logger(), "Current claw position: %.2f (0=closed, 1=open)", current_position_);
    }
    
    void position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_position_ = msg->data;
    }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr open_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr close_client_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct termios original_termios_;
    double current_position_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClawTeleop>());
    rclcpp::shutdown();
    return 0;
}
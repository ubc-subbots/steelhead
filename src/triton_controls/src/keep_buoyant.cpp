#include "triton_controls/keep_buoyant.hpp"
using std::placeholders::_1;

namespace triton_controls {   

    geometry_msgs::msg::Quaternion initialOrientation;
    bool set = false;
    bool started = false;
    bool stopped = false;
    rclcpp::Time start_time;
    double delay_seconds = 5.0;      // Delay before starting
    double run_seconds = 10.0;       // Duration to run before stopping

    /* Constructor */
    KeepBoyant::KeepBoyant(const rclcpp::NodeOptions &options)
        : Node("trajectory_generator", options)
        { 
        state_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/triton/drivers/imu/out", 10, std::bind(&KeepBoyant::state_callback, this, _1));
        pub_ = create_publisher<geometry_msgs::msg::Wrench>(
            "/triton/controls/input_forces",
            10);

        RCLCPP_INFO(this->get_logger(), "Keep Boyant successfully started!");
    }


    void KeepBoyant::state_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto now = this->get_clock()->now();

        if (!set) {
            initialOrientation = msg->orientation;
            set = true;
            start_time = now;  // Track when we first got IMU data (start of delay)
        }

        // Wait for delay before starting
        if (!started && (now - start_time).seconds() < delay_seconds) {
            return;  // Using start_time as initialization timestamp
        }

        if (!started) {
            started = true;
            start_time = now;  // RESET: Now tracks when active control begins
            RCLCPP_INFO(this->get_logger(), "Starting buoyancy control after delay.");
        }

        // Run for run_seconds, then stop
        if (!stopped && (now - start_time).seconds() < run_seconds) {
            // Using start_time as maneuver start timestamp
            geometry_msgs::msg::Wrench replyMsg;
            replyMsg.force.x = 15;

            replyMsg.torque.x = (msg->orientation.x > initialOrientation.x) ? -1 : 1;
            replyMsg.torque.y = (msg->orientation.y > initialOrientation.y) ? -1 : 1;
            replyMsg.torque.z = (msg->orientation.z > initialOrientation.z) ? -1 : 1;

            pub_->publish(replyMsg);
        } else if (!stopped) {
            // Send zero wrench and stop
            geometry_msgs::msg::Wrench stopMsg;
            pub_->publish(stopMsg);
            stopped = true;
            RCLCPP_INFO(this->get_logger(), "Stopping buoyancy control after run time.");
        }
    }
}

int main(int argc, char * argv[]) {
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::KeepBoyant>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){
    // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
  } // during testing sometimes throws error
  return 0;
}
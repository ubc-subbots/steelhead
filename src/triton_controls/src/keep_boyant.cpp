#include "triton_controls/keep_boyant.hpp"
using std::placeholders::_1;

namespace triton_controls {   

    geometry_msgs::msg::Quaternion initialOrientation;
    bool set = false;
    
    
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
            if (!set) {
                initialOrientation = msg->orientation;
                set = true;
            }

            geometry_msgs::msg::Wrench replyMsg = geometry_msgs::msg::Wrench();
            replyMsg.force.x = 15;

            if (msg->orientation.x > initialOrientation.x) {
                replyMsg.torque.x = -1;
            } else {
                replyMsg.torque.x = 1;
            }

            if (msg->orientation.y > initialOrientation.y) {
                replyMsg.torque.y = -1;
            } else {
                replyMsg.torque.y = 1;
            }

            if (msg->orientation.z > initialOrientation.z) {
                replyMsg.torque.z = -1;
            } else {
                replyMsg.torque.z = 1;
            }

            pub_->publish(replyMsg);

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
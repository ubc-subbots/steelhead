#include "triton_controls/hover_underwater.hpp"
using std::placeholders::_1;

namespace triton_controls {
//this version of the file was written right before the robot broke so it's not tested. but would like to test at the next pool test.
//tldr: added imu stabilization to the going down and going up phases

    /* Constructor */
    HoverUnderwater::HoverUnderwater(const rclcpp::NodeOptions &options)
        : Node("hover_underwater", options),
          set_(false), started_(false), stopped_(false),
          delay_seconds_(1.0), dive_seconds_(0.0), 
          hover_seconds_(30.0), surface_seconds_(3.0),
          initial_orientation_set_(false),
          kp_roll_(0.0), kp_pitch_(6.0), kp_yaw_(0.1) // roll can't work with this design, pitch needs to be higher because it's at the same time applying downard force, yaw might not be worth
        { 
        state_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/triton/drivers/imu/out", 10, std::bind(&HoverUnderwater::state_callback, this, _1));
        pub_ = create_publisher<geometry_msgs::msg::Wrench>(
            "/triton/controls/input_forces",
            10);

        RCLCPP_INFO(this->get_logger(), "Hover Underwater successfully started!");
    }


    void HoverUnderwater::state_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto now = this->get_clock()->now();

        if (!set_) {
            start_time_ = now;
            set_ = true;
        }

        double elapsed_time = (now - start_time_).seconds();

        // Set initial orientation from first reading
        if (!initial_orientation_set_) {
            initial_orientation_ = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
            initial_orientation_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial orientation set from first IMU reading");
        }

        // Wait for delay before starting
        if (!started_ && elapsed_time < delay_seconds_) {
            return;
        }

        if (!started_) {
            started_ = true;
            control_start_time_ = now;
            RCLCPP_INFO(this->get_logger(), "Starting dive phase for %.1f seconds.", dive_seconds_);
        }

        double control_elapsed = (now - control_start_time_).seconds();
        // Phase 1: Dive down with IMU stabilization
        if (!stopped_ && control_elapsed < dive_seconds_) {
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.z = -30.0;
            
            // Apply corrective forces using direct quaternion error computation
            tf2::Quaternion current_quat(msg->orientation.x, msg->orientation.y, 
                                       msg->orientation.z, msg->orientation.w);
            
            // Compute quaternion error: q_error = q_initial^-1 * q_current
            tf2::Quaternion q_error = initial_orientation_.inverse() * current_quat;
            
            // Extract rotation axis and angle from error quaternion
            tf2::Vector3 error_axis = tf2::Vector3(q_error.x(), q_error.y(), q_error.z());
            double error_angle = 2.0 * atan2(error_axis.length(), q_error.w());
            
            if (error_axis.length() > 1e-6) {
                error_axis = error_axis.normalized() * error_angle;
            }
            
            control_msg.torque.x = kp_roll_ * error_axis.x();
            control_msg.torque.y = kp_pitch_ * error_axis.y();
            control_msg.torque.z = kp_yaw_ * error_axis.z();
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Diving... %.1f seconds remaining", dive_seconds_ - control_elapsed);
            pub_->publish(control_msg);
        }

// NOTE: THIS WORKS REALLY WELL PHASE 2 is best
        // Phase 2: Hover underwater with stabilization for 15 seconds
        else if (!stopped_ && control_elapsed < (dive_seconds_ + hover_seconds_)) {
            if (control_elapsed >= dive_seconds_ && control_elapsed < dive_seconds_ + 0.1) {
                RCLCPP_INFO(this->get_logger(), "Starting hover stabilization for %.1f seconds.", hover_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            
            // Constant light downward thrust during hovering
            control_msg.force.z = -7.0; //adjust this if going down too much during hovering

            // Apply corrective forces using direct quaternion error computation
            tf2::Quaternion current_quat(msg->orientation.x, msg->orientation.y, 
                                       msg->orientation.z, msg->orientation.w);
            
            // Compute quaternion error: q_error = q_initial^-1 * q_current
            tf2::Quaternion q_error = initial_orientation_.inverse() * current_quat;
            
            // Extract rotation axis and angle from error quaternion
            tf2::Vector3 error_axis = tf2::Vector3(q_error.x(), q_error.y(), q_error.z());
            double error_angle = 2.0 * atan2(error_axis.length(), q_error.w());
            
            if (error_axis.length() > 1e-6) {
                error_axis = error_axis.normalized() * error_angle;
            }
            
            control_msg.torque.x = kp_roll_ * error_axis.x();
            control_msg.torque.y = kp_pitch_ * error_axis.y();
            control_msg.torque.z = kp_yaw_ * error_axis.z();

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Hovering stable... %.1f seconds remaining", (dive_seconds_ + hover_seconds_) - control_elapsed);
            pub_->publish(control_msg);
        }
        // Phase 3: Light upward thrust with stabilization for 3 seconds to begin surfacing
        else if (!stopped_ && control_elapsed < (dive_seconds_ + hover_seconds_ + surface_seconds_)) {
            if (control_elapsed >= (dive_seconds_ + hover_seconds_) && 
                control_elapsed < (dive_seconds_ + hover_seconds_ + 0.1)) {
                RCLCPP_INFO(this->get_logger(), "Starting surface assist for %.1f seconds.", surface_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.z = 4.0;

            // Apply corrective forces using direct quaternion error computation
            tf2::Quaternion current_quat(msg->orientation.x, msg->orientation.y, 
                                       msg->orientation.z, msg->orientation.w);
            
            tf2::Quaternion q_error = initial_orientation_.inverse() * current_quat;
            
            tf2::Vector3 error_axis = tf2::Vector3(q_error.x(), q_error.y(), q_error.z());
            double error_angle = 2.0 * atan2(error_axis.length(), q_error.w());
            
            if (error_axis.length() > 1e-6) {
                error_axis = error_axis.normalized() * error_angle;
            }
            
            control_msg.torque.x = kp_roll_ * error_axis.x();
            control_msg.torque.y = kp_pitch_ * error_axis.y();
            control_msg.torque.z = kp_yaw_ * error_axis.z();

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Surface assist... %.1f seconds remaining", 
                               (dive_seconds_ + hover_seconds_ + surface_seconds_) - control_elapsed);
            pub_->publish(control_msg);
        } 
        else if (!stopped_) {
            // Send zero wrench and stop
            geometry_msgs::msg::Wrench stop_msg;
            pub_->publish(stop_msg);
            stopped_ = true;
            RCLCPP_INFO(this->get_logger(), "Sequence complete - buoyancy will handle surfacing.");
        }
    }
    
}

int main(int argc, char * argv[]) {
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::HoverUnderwater>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){
    // Error thrown during testing sometimes
  }
  return 0;
}

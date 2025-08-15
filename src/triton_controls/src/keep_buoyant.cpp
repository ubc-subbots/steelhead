#include "triton_controls/keep_buoyant.hpp"
using std::placeholders::_1;

namespace triton_controls {

    /* Constructor */
    KeepBuoyant::KeepBuoyant(const rclcpp::NodeOptions &options)
        : Node("keep_buoyant", options),
          set_(false), started_(false), stopped_(false),
          delay_seconds_(5.0), dive_seconds_(12.0), 
          forward1_seconds_(3.0), flip_seconds_(4.0), 
          stabilize_seconds_(3.0), forward2_seconds_(8.0),
          initial_orientation_set_(false),
        //   change the strength of the correcting force by changing the numbers below
          kp_roll_(0.0), kp_pitch_(5.0), kp_yaw_(0.0) // Enable gentle pitch control to prevent up/down oscillations 
        { 
        state_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/triton/drivers/imu/out", 10, std::bind(&KeepBuoyant::state_callback, this, _1));
        pub_ = create_publisher<geometry_msgs::msg::Wrench>(
            "/triton/controls/input_forces",
            10);


        RCLCPP_INFO(this->get_logger(), "Keep Buoyant successfully started!");
    }


    void KeepBuoyant::state_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto now = this->get_clock()->now();

        if (!set_) {
            start_time_ = now;  // Track when we first got IMU data
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
            control_start_time_ = now;  // Track when active control begins
            RCLCPP_INFO(this->get_logger(), "Starting dive phase for %.1f seconds.", dive_seconds_);
        }

        double control_elapsed = (now - control_start_time_).seconds();

        // Phase 1: Dive down for dive_seconds
        if (!stopped_ && control_elapsed < dive_seconds_) {
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.z = -15.0;  // Strong downward thrust to dive
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Diving... %.1f seconds remaining", dive_seconds_ - control_elapsed);
            pub_->publish(control_msg);
        }
        // Phase 2: Forward motion for 3 seconds
        else if (!stopped_ && control_elapsed < (dive_seconds_ + forward1_seconds_)) {
            if (control_elapsed >= dive_seconds_ && control_elapsed < dive_seconds_ + 0.1) {
                RCLCPP_INFO(this->get_logger(), "Starting first forward motion phase for %.1f seconds.", forward1_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.x = 15.0;  // Forward thrust

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
            
            control_msg.torque.x = -kp_roll_ * error_axis.x();
            control_msg.torque.y = -kp_pitch_ * error_axis.y();
            control_msg.torque.z = -kp_yaw_ * error_axis.z();

            pub_->publish(control_msg);
        }
        // Phase 3: Front flip for 4 seconds
        else if (!stopped_ && control_elapsed < (dive_seconds_ + forward1_seconds_ + flip_seconds_)) {
            if (control_elapsed >= (dive_seconds_ + forward1_seconds_) && 
                control_elapsed < (dive_seconds_ + forward1_seconds_ + 0.1)) {
                RCLCPP_INFO(this->get_logger(), "Starting front flip for %.1f seconds.", flip_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            control_msg.torque.y = 15.0;  // Torque in Y for front flip
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Front flipping... %.1f seconds remaining", 
                               (dive_seconds_ + forward1_seconds_ + flip_seconds_) - control_elapsed);
            pub_->publish(control_msg);
        }
        // Phase 4: Stabilize for 3 seconds
        else if (!stopped_ && control_elapsed < (dive_seconds_ + forward1_seconds_ + flip_seconds_ + stabilize_seconds_)) {
            if (control_elapsed >= (dive_seconds_ + forward1_seconds_ + flip_seconds_) && 
                control_elapsed < (dive_seconds_ + forward1_seconds_ + flip_seconds_ + 0.1)) {
                RCLCPP_INFO(this->get_logger(), "Starting stabilization for %.1f seconds.", stabilize_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            // All forces and torques are zero for stabilization
            pub_->publish(control_msg);
        }
        // Phase 5: Final forward motion for 8 seconds
        else if (!stopped_ && control_elapsed < (dive_seconds_ + forward1_seconds_ + flip_seconds_ + stabilize_seconds_ + forward2_seconds_)) {
            if (control_elapsed >= (dive_seconds_ + forward1_seconds_ + flip_seconds_ + stabilize_seconds_) && 
                control_elapsed < (dive_seconds_ + forward1_seconds_ + flip_seconds_ + stabilize_seconds_ + 0.1)) {
                RCLCPP_INFO(this->get_logger(), "Starting final forward motion for %.1f seconds.", forward2_seconds_);
            }
            
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.x = 15.0;  // Forward thrust

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
            
            control_msg.torque.x = -kp_roll_ * error_axis.x();
            control_msg.torque.y = -kp_pitch_ * error_axis.y();
            control_msg.torque.z = -kp_yaw_ * error_axis.z();

            pub_->publish(control_msg);
        } 
        else if (!stopped_) {
            // Send zero wrench and stop
            geometry_msgs::msg::Wrench stop_msg;
            pub_->publish(stop_msg);
            stopped_ = true;
            RCLCPP_INFO(this->get_logger(), "Sequence complete - stopping all control.");
        }
    }
    
}

int main(int argc, char * argv[]) {
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::KeepBuoyant>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){
    // Error thrown during testing sometimes
  }
  return 0;
}


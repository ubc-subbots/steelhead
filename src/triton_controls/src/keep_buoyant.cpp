#include "triton_controls/keep_buoyant.hpp"
using std::placeholders::_1;

namespace triton_controls {

    /* Constructor */
    KeepBuoyant::KeepBuoyant(const rclcpp::NodeOptions &options)
        : Node("trajectory_generator", options),
          set_(false), started_(false), stopped_(false),
          delay_seconds_(5.0), run_seconds_(15.0),
          averaging_duration_(1.0), sample_count_(0),
        //   change the strength of the correcting force by changing the numbers below
          kp_roll_(0.0), kp_pitch_(0.0), kp_yaw_(0.0) // the default values that are weak are like 0.3, 0.3, and 0.2 (bc yaw is super easy). 
        { 
        state_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/triton/drivers/imu/out", 10, std::bind(&KeepBuoyant::state_callback, this, _1));
        pub_ = create_publisher<geometry_msgs::msg::Wrench>(
            "/triton/controls/input_forces",
            10);

        // Initialize accumulated orientation
        accumulated_orientation_.x = 0.0;
        accumulated_orientation_.y = 0.0;
        accumulated_orientation_.z = 0.0;
        accumulated_orientation_.w = 0.0;

        RCLCPP_INFO(this->get_logger(), "Keep Buoyant successfully started!");
    }


    void KeepBuoyant::state_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto now = this->get_clock()->now();

        if (!set_) {
            start_time_ = now;  // Track when we first got IMU data
            set_ = true;
        }

        double elapsed_time = (now - start_time_).seconds();

        // Average orientations during the first second for better initial heading
        if (elapsed_time < averaging_duration_) {
            accumulated_orientation_.x += msg->orientation.x;
            accumulated_orientation_.y += msg->orientation.y;
            accumulated_orientation_.z += msg->orientation.z;
            accumulated_orientation_.w += msg->orientation.w;
            sample_count_++;
            return;
        }

        // Finalize initial orientation average
        if (sample_count_ > 0 && elapsed_time >= averaging_duration_ && !initial_orientation_set_) {
            initial_orientation_.x = accumulated_orientation_.x / sample_count_;
            initial_orientation_.y = accumulated_orientation_.y / sample_count_;
            initial_orientation_.z = accumulated_orientation_.z / sample_count_;
            initial_orientation_.w = accumulated_orientation_.w / sample_count_;
            
            // Normalize the quaternion
            double norm = sqrt(initial_orientation_.x * initial_orientation_.x +
                             initial_orientation_.y * initial_orientation_.y +
                             initial_orientation_.z * initial_orientation_.z +
                             initial_orientation_.w * initial_orientation_.w);
            initial_orientation_.x /= norm;
            initial_orientation_.y /= norm;
            initial_orientation_.z /= norm;
            initial_orientation_.w /= norm;
            
            initial_orientation_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial orientation averaged from %d samples", sample_count_);
        }

        // Wait for delay before starting
        if (!started_ && elapsed_time < delay_seconds_) {
            return;
        }

        if (!started_) {
            started_ = true;
            control_start_time_ = now;  // Track when active control begins
            RCLCPP_INFO(this->get_logger(), "Starting buoyancy control after delay.");
        }

        double control_elapsed = (now - control_start_time_).seconds();

        // Run for run_seconds, then stop
        if (!stopped_ && control_elapsed < run_seconds_) {
            geometry_msgs::msg::Wrench control_msg;
            control_msg.force.x = 5.0;  // Forward thrust (from -15 to 15 bc we have a 5 bit binary)

            // Convert quaternions to Euler angles for proper comparison
            tf2::Quaternion current_quat(msg->orientation.x, msg->orientation.y, 
                                       msg->orientation.z, msg->orientation.w);
            tf2::Quaternion initial_quat(initial_orientation_.x, initial_orientation_.y,
                                       initial_orientation_.z, initial_orientation_.w);
            
            // Get current Euler angles
            tf2::Matrix3x3 current_matrix(current_quat);
            double current_roll, current_pitch, current_yaw;
            current_matrix.getRPY(current_roll, current_pitch, current_yaw);
            
            // Get initial Euler angles
            tf2::Matrix3x3 initial_matrix(initial_quat);
            double initial_roll, initial_pitch, initial_yaw;
            initial_matrix.getRPY(initial_roll, initial_pitch, initial_yaw);
            
            // Calculate angle differences
            double roll_error = current_roll - initial_roll;
            double pitch_error = current_pitch - initial_pitch;
            double yaw_error = current_yaw - initial_yaw;
            
            // Normalize angles to [-pi, pi]
            roll_error = atan2(sin(roll_error), cos(roll_error));
            pitch_error = atan2(sin(pitch_error), cos(pitch_error));
            yaw_error = atan2(sin(yaw_error), cos(yaw_error));
            
            // Proportional control for smooth correction
            control_msg.torque.x = -kp_roll_ * roll_error;   // Roll correction
            control_msg.torque.y = -kp_pitch_ * pitch_error; // Pitch correction
            control_msg.torque.z = -kp_yaw_ * yaw_error;     // Yaw correction

            pub_->publish(control_msg);
        } else if (!stopped_) {
            // Send zero wrench and stop
            geometry_msgs::msg::Wrench stop_msg;
            pub_->publish(stop_msg);
            stopped_ = true;
            RCLCPP_INFO(this->get_logger(), "Stopping buoyancy control after run time.");
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
    // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
  } // during testing sometimes throws error
  return 0;
}
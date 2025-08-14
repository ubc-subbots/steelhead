#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class DirectIMUController : public rclcpp::Node {
public:
    DirectIMUController() : Node("direct_imu_controller"), forward_duration_(10, 0) {
        // Subscribe to raw IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/triton/sensors/imu/bno085/data", 10,
            std::bind(&DirectIMUController::imu_callback, this, std::placeholders::_1));
        
        // Publish forces directly to thrust allocator
        force_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
            "/triton/controls/output_forces", 10);
        
        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz control loop
            std::bind(&DirectIMUController::control_loop, this));
        
        // Initialize target orientation (will be set from first IMU reading)
        target_roll_ = 0.0;
        target_pitch_ = 0.0;
        target_yaw_ = 0.0;
        calibrated_ = false;
        
        // Simple PID gains - start conservative
        kp_roll_ = 2.0;   // Roll correction strength
        kp_pitch_ = 2.0;  // Pitch correction strength  
        kp_yaw_ = 1.0;    // Yaw correction strength
        
        forward_thrust_ = 1.0;  // Constant forward thrust (kgf)
        
        // Timer for 10-second forward motion (will be set after calibration)
        stopped_ = false;
        timer_started_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Direct IMU Controller started! Hold robot steady for 3 seconds to calibrate...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_imu_ = *msg;
        imu_received_ = true;
        
        // Auto-calibrate target orientation from first few readings
        if (!calibrated_) {
            calibration_count_++;
            
            tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // Average first 60 readings (3 seconds at 20Hz)
            target_roll_ += roll;
            target_pitch_ += pitch;
            target_yaw_ += yaw;
            
            if (calibration_count_ >= 60) {
                target_roll_ /= calibration_count_;
                target_pitch_ /= calibration_count_;
                target_yaw_ /= calibration_count_;
                calibrated_ = true;
                
                // Start the 10-second timer now
                start_time_ = this->now();
                timer_started_ = true;
                
                RCLCPP_INFO(this->get_logger(), 
                    "Calibrated! Target: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f° - Starting 10-second forward run!", 
                    target_roll_ * 180.0/M_PI, 
                    target_pitch_ * 180.0/M_PI, 
                    target_yaw_ * 180.0/M_PI);
            }
        }
    }
    
    void control_loop() {
        if (!imu_received_ || !calibrated_ || !timer_started_) return;
        
        // Check if 10 seconds have elapsed since calibration
        auto elapsed = this->now() - start_time_;
        if (elapsed >= forward_duration_ && !stopped_) {
            // Stop all thrusters
            geometry_msgs::msg::Wrench stop_msg;
            stop_msg.force.x = 0.0;
            stop_msg.force.y = 0.0;
            stop_msg.force.z = 0.0;
            stop_msg.torque.x = 0.0;
            stop_msg.torque.y = 0.0;
            stop_msg.torque.z = 0.0;
            force_pub_->publish(stop_msg);
            
            stopped_ = true;
            RCLCPP_INFO(this->get_logger(), "10 seconds elapsed - STOPPING all thrusters!");
            return;
        }
        
        if (stopped_) {
            // Keep sending zero forces
            geometry_msgs::msg::Wrench stop_msg;
            force_pub_->publish(stop_msg);
            return;
        }
        
        // Extract current orientation
        tf2::Quaternion q(current_imu_.orientation.x, current_imu_.orientation.y, current_imu_.orientation.z, current_imu_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Calculate errors
        double roll_error = target_roll_ - roll;
        double pitch_error = target_pitch_ - pitch;
        double yaw_error = target_yaw_ - yaw;
        
        // Normalize yaw error to [-pi, pi]
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        // Calculate correction forces
        // From thruster config:
        // T1,T4: x=-1.0 (rear thrusters for forward)
        // T2: y=-0.7071, z=-0.7071 (right-rear angled)  
        // T3: y=+0.7071, z=-0.7071 (left-rear angled)
        // T5: z=-1.0 (bottom thruster)
        // T6: y=-1.0 (right side thruster)
        
        geometry_msgs::msg::Wrench force_msg;
        
        // Forward thrust (constant for first 10 seconds)
        force_msg.force.x = forward_thrust_;
        
        // Roll correction: use T2/T3 angled thrusters (differential z-force)
        // Positive roll error = tilt right = need left thrust = T3 more, T2 less
        force_msg.torque.x = kp_roll_ * roll_error;
        
        // Pitch correction: use T2/T3 vs T5 (z-forces front vs back)
        // Positive pitch error = nose up = need nose down force
        force_msg.torque.y = kp_pitch_ * pitch_error;
        
        // Yaw correction: use T2 vs T3 (differential y-forces)
        // Positive yaw error = turned left of target = need right turn = T2 more, T3 less
        force_msg.torque.z = kp_yaw_ * yaw_error;
        
        force_pub_->publish(force_msg);
        
        // Log status with time remaining
        auto time_remaining = forward_duration_ - elapsed;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Time: %.1fs | Roll: %.1f° (err: %.1f°) | Pitch: %.1f° (err: %.1f°) | Yaw: %.1f° (err: %.1f°)",
            time_remaining.seconds(),
            roll * 180.0/M_PI, roll_error * 180.0/M_PI,
            pitch * 180.0/M_PI, pitch_error * 180.0/M_PI, 
            yaw * 180.0/M_PI, yaw_error * 180.0/M_PI);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::Imu current_imu_;
    bool imu_received_ = false;
    bool calibrated_ = false;
    int calibration_count_ = 0;
    
    double target_roll_, target_pitch_, target_yaw_;
    double kp_roll_, kp_pitch_, kp_yaw_;
    double forward_thrust_;
    
    // Timing for 10-second run
    rclcpp::Time start_time_;
    rclcpp::Duration forward_duration_;
    bool stopped_;
    bool timer_started_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectIMUController>());
    rclcpp::shutdown();
    return 0;
}
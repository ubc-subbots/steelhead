#include "triton_controls/straight_line_navigator.hpp"

namespace triton_controls {   

    StraightLineNavigator::StraightLineNavigator(const rclcpp::NodeOptions &options)
        : Node("straight_line_navigator", options),
        navigation_state_(STRAIGHT_CALIBRATING),
        pose_initialized_(false),
        calibrated_heading_(0.0),
        compass_calibrated_(false),
        cooldown_duration_(std::chrono::milliseconds(10000)),
        navigation_distance_(20.0),
        forward_speed_(2.0)
        { 

        waypoint_publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/set", 10);
        mode_publisher_ = this->create_publisher<triton_interfaces::msg::TrajectoryType>("/triton/controls/straight_line_navigator/current_mode", 10);

        state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/triton/controls/ukf/odometry/filtered", 10, 
            std::bind(&StraightLineNavigator::state_callback, this, std::placeholders::_1));

        navigation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&StraightLineNavigator::navigation_timer_callback, this));

        state_start_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "Straight Line Navigator successfully started! Starting compass calibration...");
    }

    double StraightLineNavigator::get_yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void StraightLineNavigator::calibrate_compass() {
        if (!compass_calibrated_ && pose_initialized_) {
            calibrated_heading_ = get_yaw_from_pose(current_pose_);
            compass_calibrated_ = true;
            navigation_state_ = STRAIGHT_COOLDOWN;
            state_start_time_ = std::chrono::steady_clock::now();
            
            RCLCPP_INFO(this->get_logger(), "Compass calibrated! Current heading: %.3f radians (%.1f degrees)", 
                       calibrated_heading_, calibrated_heading_ * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "Starting 10-second cooldown period...");
        }

        // No stabilization needed during calibration - robot is manually held
    }

    void StraightLineNavigator::cooldown_wait() {
        auto elapsed = std::chrono::steady_clock::now() - state_start_time_;
        
        if (elapsed >= cooldown_duration_) {
            navigation_state_ = STRAIGHT_NAVIGATING;
            RCLCPP_INFO(this->get_logger(), "Cooldown complete! Starting straight-line navigation...");
        }

        // Continue stabilizing during cooldown - use IMU orientation only
        auto stabilize_msg = triton_interfaces::msg::Waypoint();
        stabilize_msg.pose.position.x = current_pose_.position.x;
        stabilize_msg.pose.position.y = current_pose_.position.y;
        stabilize_msg.pose.position.z = current_pose_.position.z;
        stabilize_msg.pose.orientation = current_pose_.orientation; // Use IMU orientation (roll, pitch, yaw)
        stabilize_msg.distance.position.x = 0.1;
        stabilize_msg.distance.position.y = 0.1;
        stabilize_msg.distance.position.z = 0.1;
        stabilize_msg.duration = 1;
        stabilize_msg.type = 0; // STABILIZE
        
        waypoint_publisher_->publish(stabilize_msg);

        // Log countdown every second
        auto remaining_ms = cooldown_duration_ - elapsed;
        if (remaining_ms.count() % 1000 < 100) { // Log approximately every second
            RCLCPP_INFO(this->get_logger(), "Cooldown remaining: %.1f seconds", 
                       remaining_ms.count() / 1000.0);
        }
    }

    void StraightLineNavigator::navigate_straight() {
        auto navigate_msg = triton_interfaces::msg::Waypoint();

        // Extract current roll and pitch from IMU, but keep calibrated yaw
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);
        tf2::Matrix3x3 current_m(current_q);
        double current_roll, current_pitch, current_yaw;
        current_m.getRPY(current_roll, current_pitch, current_yaw);

        // Use current roll and pitch from IMU, but calibrated yaw for straight line
        tf2::Quaternion target_orientation;
        target_orientation.setRPY(current_roll, current_pitch, calibrated_heading_);
        navigate_msg.pose.orientation = tf2::toMsg(target_orientation);

        // For position, move forward relative to current position using only the calibrated heading
        // This avoids relying on drifting x/y coordinates for navigation direction
        tf2::Vector3 forward_vector(navigation_distance_, 0, 0);
        
        // Create quaternion from calibrated heading only (ignore current roll/pitch for direction)
        tf2::Quaternion calibrated_q;
        calibrated_q.setRPY(0.0, 0.0, calibrated_heading_);
        
        // Rotate forward vector by calibrated heading
        tf2::Vector3 rotated_vector = tf2::quatRotate(calibrated_q, forward_vector);
        
        // Since position is unreliable, just use current position + compass direction
        navigate_msg.pose.position.x = current_pose_.position.x + rotated_vector.getX();
        navigate_msg.pose.position.y = current_pose_.position.y + rotated_vector.getY();
        navigate_msg.pose.position.z = current_pose_.position.z;

        // Set tolerances for waypoint achievement
        navigate_msg.distance.position.x = 1.0;
        navigate_msg.distance.position.y = 1.0;
        navigate_msg.distance.position.z = 0.5;
        navigate_msg.type = 1; // PASSTHROUGH

        waypoint_publisher_->publish(navigate_msg);
        
        // Log current status including roll and pitch from IMU
        double heading_error = current_yaw - calibrated_heading_;
        
        // Normalize heading error to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            "Navigating straight | Target: %.1f° | Current: %.1f° | Heading error: %.1f° | Roll: %.1f° | Pitch: %.1f°",
                            calibrated_heading_ * 180.0 / M_PI,
                            current_yaw * 180.0 / M_PI,
                            heading_error * 180.0 / M_PI,
                            current_roll * 180.0 / M_PI,
                            current_pitch * 180.0 / M_PI);
    }

    void StraightLineNavigator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        pose_initialized_ = true;
    }

    void StraightLineNavigator::navigation_timer_callback() {
        if (!pose_initialized_) {
            return;
        }

        // Publish current mode
        auto mode_msg = triton_interfaces::msg::TrajectoryType();
        mode_msg.type = navigation_state_;
        mode_publisher_->publish(mode_msg);

        switch (navigation_state_) {
            case STRAIGHT_CALIBRATING:
                calibrate_compass();
                break;
                
            case STRAIGHT_COOLDOWN:
                cooldown_wait();
                break;
                
            case STRAIGHT_NAVIGATING:
                navigate_straight();
                break;
                
            case STRAIGHT_COMPLETED:
                {
                    // Stabilize in place using IMU orientation
                    auto stop_msg = triton_interfaces::msg::Waypoint();
                    stop_msg.pose.position.x = current_pose_.position.x;
                    stop_msg.pose.position.y = current_pose_.position.y;
                    stop_msg.pose.position.z = current_pose_.position.z;
                    stop_msg.pose.orientation = current_pose_.orientation; // Use IMU orientation
                    stop_msg.distance.position.x = 0.1;
                    stop_msg.distance.position.y = 0.1;
                    stop_msg.distance.position.z = 0.1;
                    stop_msg.duration = 5;
                    stop_msg.type = 0; // STABILIZE
                    waypoint_publisher_->publish(stop_msg);
                    
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                        "Navigation completed. Maintaining position.");
                }
                break;
        }
    }

} // namespace triton_controls

int main(int argc, char * argv[]) {
    try {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions();
        rclcpp::spin(std::make_shared<triton_controls::StraightLineNavigator>(options));
        rclcpp::shutdown();
    } catch (rclcpp::exceptions::RCLError const&) {
        // Error handling
    }
    return 0;
}
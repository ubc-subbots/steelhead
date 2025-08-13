#include "triton_controls/simple_gate_navigator.hpp"

namespace triton_controls {   

    SimpleGateNavigator::SimpleGateNavigator(const rclcpp::NodeOptions &options)
        : Node("simple_gate_navigator", options),
        gate_state_(GATE_SEARCHING),
        gate_detected_(false),
        pose_initialized_(false),
        navigation_distance_(10.0),
        gate_loss_timeout_(std::chrono::milliseconds(3000)),
        thrust_duration_(std::chrono::milliseconds(10000)),
        gate_ever_detected_(false)
        { 

        waypoint_publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/set", 10);
        mode_publisher_ = this->create_publisher<triton_interfaces::msg::TrajectoryType>("/triton/controls/simple_gate_navigator/current_mode", 10);

        state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/triton/controls/ukf/odometry/filtered", 10, 
            std::bind(&SimpleGateNavigator::state_callback, this, std::placeholders::_1));

        gate_subscription_ = this->create_subscription<triton_interfaces::msg::ObjectOffset>(
            "/triton/gate/detector/gate_pose", 10, 
            std::bind(&SimpleGateNavigator::gate_callback, this, std::placeholders::_1));

        navigation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&SimpleGateNavigator::navigation_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Simple Gate Navigator successfully started!");
    }

    double SimpleGateNavigator::get_yaw_from_pose(const geometry_msgs::msg::Pose& pose) {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double SimpleGateNavigator::calculate_distance_to_gate() {
        if (!gate_detected_) {
            return -1.0;
        }

        return std::sqrt(std::pow(gate_pose_.position.x, 2) + std::pow(gate_pose_.position.y, 2));
    }

    void SimpleGateNavigator::search_for_gate() {
        auto search_msg = triton_interfaces::msg::Waypoint();
        search_msg.pose = current_pose_;

        double current_yaw = get_yaw_from_pose(current_pose_);
        
        tf2::Quaternion search_orientation;
        search_orientation.setRPY(0.0, 0.0, current_yaw + 0.2);
        search_msg.pose.orientation = tf2::toMsg(search_orientation);

        search_msg.distance.position.x = 0.3;
        search_msg.distance.position.y = 0.3;
        search_msg.distance.position.z = 0.3;
        search_msg.duration = 1;
        search_msg.type = 0;

        waypoint_publisher_->publish(search_msg);
    }

    void SimpleGateNavigator::approach_gate() {
        auto approach_msg = triton_interfaces::msg::Waypoint();

        double gate_yaw = std::atan2(gate_pose_.position.y, gate_pose_.position.x);
        double current_yaw = get_yaw_from_pose(current_pose_);
        double target_yaw = current_yaw + gate_yaw;

        tf2::Quaternion target_orientation;
        target_orientation.setRPY(0.0, 0.0, target_yaw);
        approach_msg.pose.orientation = tf2::toMsg(target_orientation);

        // Move forward toward the gate - use a fixed forward distance for steady approach
        double approach_forward_distance = 2.0; // Always move 2 meters forward
        
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);
        
        tf2::Vector3 forward_vector(approach_forward_distance, 0, 0);
        tf2::Vector3 rotated_vector = tf2::quatRotate(current_q, forward_vector);
        
        approach_msg.pose.position.x = current_pose_.position.x + rotated_vector.getX();
        approach_msg.pose.position.y = current_pose_.position.y + rotated_vector.getY();
        approach_msg.pose.position.z = current_pose_.position.z + gate_pose_.position.z;

        approach_msg.distance.position.x = 0.5;
        approach_msg.distance.position.y = 1.5;
        approach_msg.distance.position.z = 1.0;
        approach_msg.type = 1;

        waypoint_publisher_->publish(approach_msg);
        
        RCLCPP_INFO(this->get_logger(), "Approaching gate. Distance: %.2f", calculate_distance_to_gate());
    }

    void SimpleGateNavigator::navigate_through_gate() {
        auto navigate_msg = triton_interfaces::msg::Waypoint();

        // Just go straight forward from current position
        double current_yaw = get_yaw_from_pose(current_pose_);

        tf2::Quaternion target_orientation;
        target_orientation.setRPY(0.0, 0.0, current_yaw);
        navigate_msg.pose.orientation = tf2::toMsg(target_orientation);

        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);
        
        tf2::Vector3 forward_vector(navigation_distance_, 0, 0);
        tf2::Vector3 rotated_vector = tf2::quatRotate(current_q, forward_vector);
        
        navigate_msg.pose.position.x = current_pose_.position.x + rotated_vector.getX();
        navigate_msg.pose.position.y = current_pose_.position.y + rotated_vector.getY();
        navigate_msg.pose.position.z = current_pose_.position.z;

        navigate_msg.distance.position.x = 0.5;
        navigate_msg.distance.position.y = 0.5;
        navigate_msg.distance.position.z = 0.5;
        navigate_msg.type = 1;

        waypoint_publisher_->publish(navigate_msg);
        RCLCPP_INFO(this->get_logger(), "Thrusting straight forward...");
    }

    void SimpleGateNavigator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        pose_initialized_ = true;
    }

    void SimpleGateNavigator::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg) {
        if (msg->class_id == 1) {
            if (abs(msg->pose.position.x) < 50 && 
                abs(msg->pose.position.y) < 50 && 
                abs(msg->pose.position.z) < 20) {
                
                gate_pose_ = msg->pose;
                gate_detected_ = true;
                gate_ever_detected_ = true;
                last_gate_seen_time_ = std::chrono::steady_clock::now();

                if (gate_state_ == GATE_SEARCHING) {
                    gate_state_ = GATE_APPROACHING;
                    RCLCPP_INFO(this->get_logger(), "Gate detected! Starting approach.");
                }
            }
        }
    }

    void SimpleGateNavigator::navigation_timer_callback() {
        if (!pose_initialized_) {
            return;
        }

        auto mode_msg = triton_interfaces::msg::TrajectoryType();
        mode_msg.type = gate_state_;
        mode_publisher_->publish(mode_msg);

        switch (gate_state_) {
            case GATE_SEARCHING:
                search_for_gate();
                break;
                
            case GATE_APPROACHING:
                if (gate_ever_detected_) {
                    // Check if we've lost sight of the gate for 3 seconds
                    auto time_since_last_gate = std::chrono::steady_clock::now() - last_gate_seen_time_;
                    if (time_since_last_gate > gate_loss_timeout_) {
                        // Lost sight of gate for 3 seconds - start 5 second thrust
                        gate_state_ = GATE_THRUSTING;
                        thrust_start_time_ = std::chrono::steady_clock::now();
                        RCLCPP_INFO(this->get_logger(), "Lost sight of gate for 3 seconds! Starting 5-second straight thrust.");
                    } else {
                        // Still see the gate recently - keep approaching
                        approach_gate();
                    }
                } else {
                    // Haven't found gate yet - keep searching
                    search_for_gate();
                }
                break;
                
            case GATE_THRUSTING:
                {
                    auto elapsed = std::chrono::steady_clock::now() - thrust_start_time_;
                    if (elapsed < thrust_duration_) {
                        navigate_through_gate();
                    } else {
                        gate_state_ = GATE_COMPLETED;
                        RCLCPP_INFO(this->get_logger(), "5-second thrust completed! Navigation finished.");
                    }
                }
                break;
                
            case GATE_COMPLETED:
                {
                    auto stop_msg = triton_interfaces::msg::Waypoint();
                    stop_msg.pose = current_pose_;
                    stop_msg.distance.position.x = 0.1;
                    stop_msg.distance.position.y = 0.1;
                    stop_msg.distance.position.z = 0.1;
                    stop_msg.duration = 5;
                    stop_msg.type = 0;
                    waypoint_publisher_->publish(stop_msg);
                }
                break;
        }
    }

} // namespace triton_controls

int main(int argc, char * argv[]) {
    try {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions();
        rclcpp::spin(std::make_shared<triton_controls::SimpleGateNavigator>(options));
        rclcpp::shutdown();
    } catch (rclcpp::exceptions::RCLError const&) {
        // Error handling
    }
    return 0;
}
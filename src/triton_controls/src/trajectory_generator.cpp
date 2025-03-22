#include "triton_controls/trajectory_generator.hpp"
using std::placeholders::_1;

namespace triton_controls {   
    // GLOBAL VARIABLES
    bool approach_destination_achieved_ = false; // whether we have successfully closed the distance to the buoy
    double radius_for_rotation = 1.0; 
    double distance_to_buoy = 999999;
    bool rotationStart = true;
        /* Constructor */
    TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions &options)
        : Node("trajectory_generator", options),
        type_(TRAJ_START),
        destination_achieved_(true),
        buoy_state_(BUOY_UNINITIALIZED)
        { 


        waypoint_publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/set", 10);

        state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/triton/controls/ukf/odometry/filtered", 10, std::bind(&TrajectoryGenerator::state_callback, this, _1));

        type_subscription_ = this->create_subscription<triton_interfaces::msg::TrajectoryType>(
            "/triton/controls/trajectory_generator/set_type", 10, std::bind(&TrajectoryGenerator::type_callback, this, _1));

        gate_subscription_ = this->create_subscription<triton_interfaces::msg::ObjectOffset>(
            "/triton/gate/detector/gate_pose", 10, std::bind(&TrajectoryGenerator::gate_callback, this, _1));

        waypoint_subscription_ = this->create_subscription<triton_interfaces::msg::Waypoint>(
            "/triton/controls/waypoint_marker/current_goal", 10, std::bind(&TrajectoryGenerator::waypoint_callback, this, _1));

        // this->declare_parameter("start_turning_factor", start_turning_factor_);
        // this->get_parameter("start_turning_factor", start_turning_factor_);

        RCLCPP_INFO(this->get_logger(), "Trajectory Generator successfully started!");
    }

    void TrajectoryGenerator::approach_gate() {
        auto reply_msg = triton_interfaces::msg::Waypoint(); // Create a new waypoint message

        // Prepare to calculate adjustments for the waypoint
        // reply_msg is in the map frame, destination_pose_ is in the base frame, and current_pose_ is in the map frame

        // If the gate is not directly in front, calculate yaw to turn toward it
        double required_yaw = std::atan(destination_pose_.position.y / destination_pose_.position.x); // Calculate yaw adjustment

        // Extract the current pose's orientation as a quaternion
        tf2::Quaternion current_pose_q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 current_pose_q_m(current_pose_q); // Convert quaternion to a rotation matrix
        double current_pose_roll, current_pose_pitch, current_pose_yaw; // Variables to store roll, pitch, yaw
        current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw); // Extract roll, pitch, yaw

        // Set the new orientation based on the required yaw
        tf2::Quaternion tf2_quat_destination;
        tf2_quat_destination.setRPY(current_pose_roll, current_pose_pitch, current_pose_yaw + required_yaw); // Add yaw adjustment
        reply_msg.pose.orientation.x = tf2_quat_destination.x(); // Set the waypoint orientation (x component)
        reply_msg.pose.orientation.y = tf2_quat_destination.y(); // Set the waypoint orientation (y component)
        reply_msg.pose.orientation.z = tf2_quat_destination.z(); // Set the waypoint orientation (z component)
        reply_msg.pose.orientation.w = tf2_quat_destination.w(); // Set the waypoint orientation (w component)

        // Calculate distance to gate and position waypoint accordingly
        double distance_x = std::sqrt(std::pow(destination_pose_.position.y, 2) + std::pow(destination_pose_.position.x, 2)); // Calculate Euclidean distance
        tf2::Quaternion current_q; // Create a quaternion for rotation
        tf2::Vector3 dest_v; // Create a vector for destination
        dest_v.setX(distance_x + 1); // Set forward distance with buffer
        dest_v.setY(0); // No lateral offset
        dest_v.setZ(0); // No vertical offset
        tf2::fromMsg(current_pose_.orientation, current_q); // Convert current orientation to quaternion
        tf2::Vector3 targetForward = tf2::quatRotate(current_q, dest_v); // Rotate vector to align with AUV heading
        reply_msg.pose.position.x = current_pose_.position.x + targetForward.getX(); // Set waypoint position x
        reply_msg.pose.position.y = current_pose_.position.y + targetForward.getY(); // Set waypoint position y
        reply_msg.pose.position.z = current_pose_.position.z + destination_pose_.position.z; // Adjust waypoint position z

        // Define a fixed distance tolerance
        tf2::Quaternion tf2_quat_distance;
        tf2_quat_distance.setRPY(1.57, 1.57, 1.57); // Define rotational distance tolerances
        reply_msg.distance.position.x = 0.5; // Set forward tolerance
        reply_msg.distance.position.y = 4.0; // Set lateral tolerance for gate width
        reply_msg.distance.position.z = 2.0; // Set vertical tolerance for gate height
        reply_msg.distance.orientation.x = tf2_quat_distance.x(); // Set orientation distance (x component)
        reply_msg.distance.orientation.y = tf2_quat_distance.y(); // Set orientation distance (y component)
        reply_msg.distance.orientation.z = tf2_quat_distance.z(); // Set orientation distance (z component)
        reply_msg.distance.orientation.w = tf2_quat_distance.w(); // Set orientation distance (w component)
        reply_msg.type = 1; // PASSTHROUGH type for the waypoint
        waypoint_publisher_->publish(reply_msg); // Publish the waypoint message
    }

    void TrajectoryGenerator::aim_back_at_start() {
        auto reply_msg = triton_interfaces::msg::Waypoint();

        // Set waypoint to the starting position
        reply_msg.pose.position = starting_position_;

        // Calculate orientation to face the starting position
        double dx = starting_position_.x - current_pose_.position.x;
        double dy = starting_position_.y - current_pose_.position.y;
        double yaw_to_start = atan2(dy, dx);
        tf2::Quaternion waypoint_orientation;
        waypoint_orientation.setRPY(0, 0, yaw_to_start);
        reply_msg.pose.orientation = tf2::toMsg(waypoint_orientation);

        // Set tolerances
        reply_msg.distance.position.x = 0.5;
        reply_msg.distance.position.y = 0.5;
        reply_msg.distance.position.z = 0.5;

        // Set duration and type
        reply_msg.duration = 5;  
        reply_msg.type = 0;  // STABILIZE

        waypoint_publisher_->publish(reply_msg);
    }

    tf2::Vector3 TrajectoryGenerator::get_buoy_global_position() {
        // 1. Convert the AUV's current quaternion to a tf2::Quaternion
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);

        // 2. The buoy offset in the AUV’s local frame
        tf2::Vector3 local_offset(
            -13,
            0,
            1
        );

        // 3. Rotate local_offset by current_q to get the buoy offset in the GLOBAL frame
        tf2::Vector3 global_offset = tf2::quatRotate(current_q, local_offset);

        // 4. Now add that global offset to the AUV’s current global position
        tf2::Vector3 buoy_global_pos(
            current_pose_.position.x + global_offset.x(),
            current_pose_.position.y + global_offset.y(),
            current_pose_.position.z + global_offset.z()
        );
        return local_offset;
    }

    void TrajectoryGenerator::rotate_around_buoy() {
    int num_waypoints = 12; // Number of waypoints for the circle
    double radius = radius_for_rotation; // Desired radius for rotation
    double angle_step = 2 * M_PI / num_waypoints; // Equal angle steps
    double offset_distance = 3.0; // Distance of imaginary point from current pose

    // Get current yaw
    tf2::Quaternion current_q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w);
    tf2::Matrix3x3 m(current_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Compute the new center of rotation (3 meters in front)
    double center_x = current_pose_.position.x + offset_distance * cos(yaw);
    double center_y = current_pose_.position.y + offset_distance * sin(yaw);
    double center_z = current_pose_.position.z; // Maintain depth

    // Stabilize first
    auto stop_msg = triton_interfaces::msg::Waypoint();
    stop_msg.pose = current_pose_;  // Keep the current position
    stop_msg.pose.orientation = current_pose_.orientation;
    waypoint_publisher_->publish(stop_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Time to stabilize

    // Generate circular waypoints around the new center
    for (int i = 0; i < num_waypoints; ++i) {
        double angle = i * angle_step;

        // Calculate waypoint positions relative to the new center
        double waypoint_x = center_x + radius * cos(angle);
        double waypoint_y = center_y + radius * sin(angle);
        double waypoint_z = center_z; // Maintain depth

        // Set waypoint message
        auto reply_msg = triton_interfaces::msg::Waypoint();
        reply_msg.pose.position.x = waypoint_x;
        reply_msg.pose.position.y = waypoint_y;
        reply_msg.pose.position.z = waypoint_z;

        // Face toward the center of rotation
        double new_yaw = atan2(center_y - waypoint_y, center_x - waypoint_x);
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, new_yaw);
        reply_msg.pose.orientation = tf2::toMsg(orientation);

        // Set tolerances
        reply_msg.distance.position.x = 0.5;
        reply_msg.distance.position.y = 0.5;

        // Publish waypoint
        waypoint_publisher_->publish(reply_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(10000)); // Wait for movement

        // Debug logs
        RCLCPP_INFO(this->get_logger(), "Waypoint %d: (%f, %f, %f) facing center (%f, %f) with yaw: %f",
                    i, waypoint_x, waypoint_y, waypoint_z, center_x, center_y, new_yaw);
    }

    // After completing the loop, return to start position
    current_state_ = State::RETURN_TO_START; // Transition to return state
}


    void TrajectoryGenerator:: stabilize_and_prepare_rot() {
                // Stop residual movement and stabilize
        // tf2::Quaternion current_q(
        //     current_pose_.orientation.x,
        //     current_pose_.orientation.y,
        //     current_pose_.orientation.z,
        //     current_pose_.orientation.w);

        // tf2::Matrix3x3 m(current_q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);

        // // Create a quaternion with roll and pitch set to 0, but keep the yaw
        // tf2::Quaternion level_q;
        // level_q.setRPY(0, 0, yaw);

        // // Create the stop message
        // auto stop_msg = triton_interfaces::msg::Waypoint();
        // stop_msg.pose = current_pose_;  // Keep the current position

        // stop_msg.pose.orientation.x = level_q.x();
        // stop_msg.pose.orientation.y = level_q.y();
        // stop_msg.pose.orientation.z = level_q.z();
        // stop_msg.pose.orientation.w = level_q.w();

        // // Publish the stop command
        // waypoint_publisher_->publish(stop_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(3000));
        // stop_msg.pose.position.z = 1;  // Keep the current position
        // waypoint_publisher_->publish(stop_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(3000));        I
    }

    void TrajectoryGenerator::spin_for_gate_detect(const geometry_msgs::msg::Pose msg) {
        // RCLCPP_INFO(this->get_logger(), "Scanning for targets in TRAJ_START mode.");
            // Turn the AUV around slowly (to search for gate)
        auto reply_msg = triton_interfaces::msg::Waypoint(); // Create a new waypoint message
        reply_msg.pose = msg; // Set the waypoint's pose to the current pose from odometry

        // Extract the current pose's orientation as a quaternion
        tf2::Quaternion current_pose_q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 current_pose_q_m(current_pose_q); // Convert quaternion to a rotation matrix
        double current_pose_roll, current_pose_pitch, current_pose_yaw; // Variables to store roll, pitch, yaw
        current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw); // Extract roll, pitch, yaw

        // Set some small yaw offset to slowly rotate the AUV
        tf2::Quaternion tf2_quat_dest;
        tf2_quat_dest.setRPY(0.001, 0.001, current_pose_yaw - 0.50); // Adjust yaw to rotate slightly
        reply_msg.pose.orientation.x = tf2_quat_dest.x(); // Set the waypoint orientation (x component)
        reply_msg.pose.orientation.y = tf2_quat_dest.y(); // Set the waypoint orientation (y component)
        reply_msg.pose.orientation.z = tf2_quat_dest.z(); // Set the waypoint orientation (z component)
        reply_msg.pose.orientation.w = tf2_quat_dest.w(); // Set the waypoint orientation (w component)

        // Set some small distance to move forward
        tf2::Quaternion tf2_quat_distance;
        tf2_quat_distance.setRPY(0.05, 0.05, 0.1); // Add a small positional offset

        reply_msg.distance.position.x = 0.2; // Set forward distance (x-axis)
        reply_msg.distance.position.y = 0.2; // Set lateral distance (y-axis)
        reply_msg.distance.position.z = 0.2; // Set vertical distance (z-axis)
        reply_msg.distance.orientation.x = tf2_quat_distance.x(); // Set orientation distance (x component)
        reply_msg.distance.orientation.y = tf2_quat_distance.y(); // Set orientation distance (y component)
        reply_msg.distance.orientation.z = tf2_quat_distance.z(); // Set orientation distance (z component)
        reply_msg.distance.orientation.w = tf2_quat_distance.w(); // Set orientation distance (w component)
        reply_msg.duration = 2; // Set duration for this waypoint
        reply_msg.type = 0; // STABILIZE type for the waypoint

        waypoint_publisher_->publish(reply_msg); // Publish the waypoint message
    }

    

    void TrajectoryGenerator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        current_pose_ = msg->pose.pose; // imu data being used

        switch (current_state_) {
          case State::IDLE:
              spin_for_gate_detect(current_pose_);
              // RCLCPP_INFO(this->get_logger(), "IDLE");
              break;
          
          case State::APPROACH_GATE:
              // RCLCPP_INFO(this->get_logger(), "Approaching gate");
              approach_gate();
              break;
          case State::ROTATE_AROUND_CENTER:
              // moveOneMeter();
              // RCLCPP_INFO(this->get_logger(), "rotating to start");
                // need to stabilize (yaw/roll/pitcch should be level)
                // need to be at depth 1
                // then go straight for 2 meters
                // stabilize_and_prepare_rot();
              rotate_around_buoy();
              break;
          
          case State::RETURN_TO_START:
               // RCLCPP_INFO(this->get_logger(), "Returning to start");
              aim_back_at_start();
              break;
          
          case State::ERROR:
              // Handle error (could include some recovery or retry mechanism)
              current_state_ = State::IDLE;
              // RCLCPP_INFO(this->get_logger(), "IDLING due to error ");
              break;
      }
    }


    void TrajectoryGenerator::type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg) {

        if (!(buoy_state_ == BUOY_ROTATE) && !(type_ == TRAJ_BUOY)) {
            type_ = msg->type;
            destination_achieved_ = false;

            RCLCPP_INFO(this->get_logger(), "Trajectory Type updated. ");
        }

    }

    void TrajectoryGenerator::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg) {
        /**
         * Below is the buoy_callback function implementation that currently lives in gate_callback cause I'm treating a gate like a buoy
         */

        // "turn off" gate detection when doing unrealtd things
        if (current_state_ != State::APPROACH_GATE && current_state_ != State::IDLE) {
          return;
        }

        // Check if the detected object is a gate (change to buoy later)
        if (msg->class_id == TRAJ_GATE && current_state_ != State::ROTATE_AROUND_CENTER)
        {

            // Validate the detected buoy position to avoid extreme values
            if (abs(msg->pose.position.x) < 50 
                && abs(msg->pose.position.y) < 50
                && abs(msg->pose.position.z) < 20) {
                // Update the destination pose with the detected buoy position
                destination_pose_ = msg->pose;
                
                if (current_state_ != State::APPROACH_GATE && current_state_ != State::ROTATE_AROUND_CENTER) {
                    current_state_ = State::APPROACH_GATE;
                    destination_achieved_ = false;
                    starting_position_ = current_pose_.position;  // Store the starting position
                    RCLCPP_INFO(this->get_logger(), "Gate detected! Switching to APPROACH_GATE mode and beginning approach.");
                }
                        // Check if the robot is close enough to the gate (distance < 1 meter)
                double distance = std::sqrt(
                    std::pow(destination_pose_.position.x, 2)
                );

                if (distance < 1.0) {
                    // The robot has passed through the gate
                    if (current_state_ != State::ROTATE_AROUND_CENTER) {
                        buoy_global_position_ = get_buoy_global_position(); // gets wrong position
                        current_state_ = State::ROTATE_AROUND_CENTER;
                        RCLCPP_INFO(this->get_logger(), "Gate passed! Switching to ROTATE_AROUND_CENTER mode.");
                    }
                }

                // RCLCPP_INFO(this->get_logger(), "type: %d buoy_state: %d", mode_msg.type, buoy_state_);
            } else {

            }
        } else {

        }

      /**
       * Below is the original gate_callback function that is temporarily removed
       */
      // Check if the detected object is a gate
      // if (msg->class_id == TRAJ_GATE)
      // {
      //     // Validate the detected gate position to avoid extreme values
      //     if (abs(msg->pose.position.x) < 50 
      //         && abs(msg->pose.position.y) < 50
      //         && abs(msg->pose.position.z) < 20)
      //     {
      //         // Update the destination pose with the detected gate position
      //         destination_pose_ = msg->pose;

      //         // If currently in TRAJ_START mode, switch to TRAJ_GATE mode
      //         if (type_ == TRAJ_START)
      //         {
      //             type_ = TRAJ_GATE;
      //             destination_achieved_ = false;

      //             RCLCPP_INFO(this->get_logger(), "Gate detected! Switching to TRAJ_GATE mode.");

      //             // Publish the updated mode
      //             auto mode_msg = triton_interfaces::msg::TrajectoryType();
      //             mode_msg.type = type_;
      //             current_mode_publisher_->publish(mode_msg);
      //         }
      //         else if (type_ == TRAJ_GATE)
      //         {
      //             // If already in TRAJ_GATE mode, update the destination
      //             destination_achieved_ = false;
      //         }
      //     }
      // }
  }

    void TrajectoryGenerator::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "IN WAYPOINT CALLBACK type: %d buoy_state: %d initial_rot_heading_reached_: %d", type_, buoy_state_,initial_rot_heading_reached_);
        // if (msg->success) {
        //         RCLCPP_INFO(this->get_logger(), "TEST1");
        //     if (type_ == TRAJ_BUOY) {
        //         RCLCPP_INFO(this->get_logger(), "TEST2");

        //         switch (buoy_state_) {
        //             case BUOY_APPROACH:
        //                 RCLCPP_INFO(this->get_logger(), "TEST3");
        //                 approach_destination_achieved_ = true;
        //                 RCLCPP_INFO(this->get_logger(), "Waypoint reached in BUOY_APPROACH state.");
        //                 break;
        //             case BUOY_ROTATE:
        //                 // RCLCPP_INFO(this->get_logger(), "BUOY_ROTATE_STATE");
        //                 // if (!initial_rot_heading_reached_) {
        //                 //     initial_rot_heading_reached_ = true;
        //                 //     RCLCPP_INFO(this->get_logger(), "Initial heading adjustment completed. Starting rotation.");
        //                 // } else {
        //                 //     RCLCPP_INFO(this->get_logger(), "Waypoint reached in BUOY_ROTATE state.");
        //                 // }
        //                 break;
        //             case BUOY_RETURN:
        //                 rotate_destination_achieved_ = true;
        //                 RCLCPP_INFO(this->get_logger(), "Waypoint reached in BUOY_RETURN state.");
        //                 break;
        //         }
        //     }
        // }
    }

    
}// namespace triton_controls

int main(int argc, char * argv[]) {
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::TrajectoryGenerator>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){
    // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
  } // during testing sometimes throws error
  return 0;
}
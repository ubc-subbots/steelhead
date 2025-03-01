#include "triton_controls/trajectory_generator.hpp"
using std::placeholders::_1;

namespace triton_controls {   
    // GLOBAL VARIABLES
    bool approach_destination_achieved_ = false; // whether we have successfully closed the distance to the buoy
    double radius_for_rotation = 3.0; // the destired radius of rotation (doubles as approach distance)
    double distance_to_buoy = 999999; // estimated distance to the buy
    
        /* Constructor */
    TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions &options)
        : Node("trajectory_generator", options),
        type_(TRAJ_START),
        destination_achieved_(true),
        buoy_state_(BUOY_UNINITIALIZED),
        initial_rot_heading_adjusted_(false),
        initial_rot_heading_reached_(false)
        { 

        //can remove later but this publishes the current mode
        current_mode_publisher_ = this->create_publisher<triton_interfaces::msg::TrajectoryType>("/triton/controls/trajectory_generator/current_mode", 10);


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

    // we hardcode a global path around the buoy based on the last time we estimated our position from the buoy before entering the rotation state
    void TrajectoryGenerator::rotate_around_buoy() {
        int num_waypoints = 12; // Number of waypoints for the circle
        double radius = radius_for_rotation; // Desired radius for rotation
        double angle_step = 2 * M_PI / num_waypoints; // Equal angle steps

        // Stop residual movement and stabilize
        auto stop_msg = triton_interfaces::msg::Waypoint();
        stop_msg.pose.position = current_pose_.position; // Stay in place
        stop_msg.pose.orientation = current_pose_.orientation; // Keep current orientation

        stop_msg.distance.position.x = 0.1; // Small tolerance
        stop_msg.distance.position.y = 0.1;
        stop_msg.distance.position.z = 0.1;

        waypoint_publisher_->publish(stop_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(1000)); // time to stabilize

        // Extract buoy position
        double buoy_x = buoy_global_position_.x();
        double buoy_y = buoy_global_position_.y();
        double buoy_z = buoy_global_position_.z();

        for (int i = 0; i < num_waypoints; ++i) {
            double angle = i * angle_step;

            // Calculate waypoint positions relative to the buoy
            double waypoint_x = buoy_global_position_.x() + radius * cos(angle);
            double waypoint_y = buoy_global_position_.y() + radius * sin(angle);
            double waypoint_z = buoy_global_position_.z(); // Maintain depth

            // Set waypoint message
            auto reply_msg = triton_interfaces::msg::Waypoint();
            reply_msg.pose.position.x = waypoint_x;
            reply_msg.pose.position.y = waypoint_y;
            reply_msg.pose.position.z = waypoint_z;

            // Orientation to face the buoy **can possibly remove this if we are hardcoding the path***
            // If we don't really care about the orientation to face the buoy can do a different yaw
            double yaw = atan2(buoy_global_position_.y() - waypoint_y,
                            buoy_global_position_.x() - waypoint_x);

            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, yaw);
            reply_msg.pose.orientation = tf2::toMsg(orientation);

            // Set tolerances
            reply_msg.distance.position.x = 0.5;
            reply_msg.distance.position.y = 0.5;
            reply_msg.distance.position.z = 0.5;

            // Publish waypoint
            waypoint_publisher_->publish(reply_msg);

            // Pause briefly for motion
            rclcpp::sleep_for(std::chrono::milliseconds(1000));

            // Debug logs
            RCLCPP_INFO(this->get_logger(), "Waypoint %d: (%f, %f, %f) facing buoy (%f, %f) with yaw: %f",
                        i, waypoint_x, waypoint_y, waypoint_z,
                        buoy_global_position_.x(), buoy_global_position_.y(), yaw);
        }


        // After completing the loop, return to start position (change to buoy return state)
        buoy_state_ = BUOY_RETURN; // Transition to return state
    }


    //TODO: Fix this to correctly estimate global position
    tf2::Vector3 TrajectoryGenerator::get_buoy_global_position() {
        // 1. Convert the AUV's current quaternion to a tf2::Quaternion
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);

        // 2. The buoy offset in the AUV’s local frame
        tf2::Vector3 local_offset(
            destination_pose_.position.x,
            destination_pose_.position.y,
            destination_pose_.position.z
        );

        // 3. Rotate local_offset by current_q to get the buoy offset in the GLOBAL frame
        tf2::Vector3 global_offset = tf2::quatRotate(current_q, local_offset);

        // 4. Now add that global offset to the AUV’s current global position
        tf2::Vector3 buoy_global_pos(
            current_pose_.position.x + global_offset.x(),
            current_pose_.position.y + global_offset.y(),
            current_pose_.position.z + global_offset.z()
        );

        return buoy_global_pos;
    }


    double TrajectoryGenerator::calculate_distance_to_buoy() {
        // If no destination set
        if (destination_pose_.position.x == 0 && 
            destination_pose_.position.y == 0 && 
            destination_pose_.position.z == 0) 
        {
            return -1.0;
        }

        // Convert AUV's current orientation to tf2
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);

        // The buoy offset in the AUV’s local (base) frame
        tf2::Vector3 local_offset(
            destination_pose_.position.x,
            destination_pose_.position.y,
            destination_pose_.position.z
        );

        // Rotate local_offset by current_q to get the offset in the global/map frame
        tf2::Vector3 global_offset = tf2::quatRotate(current_q, local_offset);

        // Add that global offset to the AUV's global position
        tf2::Vector3 buoy_position_in_map(
            current_pose_.position.x + global_offset.x(),
            current_pose_.position.y + global_offset.y(),
            current_pose_.position.z + global_offset.z()
        );

        // Return the distance
        // Compute 2D distance (ignore z component)
        return std::hypot(
            buoy_position_in_map.x() - auv_position.x(),
            buoy_position_in_map.y() - auv_position.y()
        );

    }


    void TrajectoryGenerator::approach_buoy() {
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

    void TrajectoryGenerator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        current_pose_ = msg->pose.pose; // imu data being used
        auto mode_msg = triton_interfaces::msg::TrajectoryType();
        mode_msg.type = type_;
        current_mode_publisher_->publish(mode_msg);

        if (type_ == TRAJ_START) {
            RCLCPP_INFO(this->get_logger(), "Scanning for targets in TRAJ_START mode.");
            // Turn the AUV around slowly (to search for gate)
            auto reply_msg = triton_interfaces::msg::Waypoint(); // Create a new waypoint message
            reply_msg.pose = msg->pose.pose; // Set the waypoint's pose to the current pose from odometry

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
        else if (type_ == TRAJ_GATE) {// Check if the mode is TRAJ_GATE 
            

            // RCLCPP_INFO(this->get_logger(), "Approaching gate in TRAJ_GATE mode. [SHOULD BE DISABLED RIGHT NOW BECAUSE I AM IMPLEMENTING BUOY NAVIGATION]");
            // TODO: Generate trajectory logic for navigating through a gate
            if (!destination_achieved_) {// If the destination hasn't been reached
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
            else // If the destination has been achieved or is invalid
            {
                RCLCPP_INFO(this->get_logger(), "Destination achieved. Continuing in TRAJ_GATE mode.");
                // Prevent division by zero in calculations
                if (destination_pose_.position.x == 0)
                {
                destination_pose_.position.x = 0.1; // Assign a small value to prevent division by zero
                }

                // Turn the AUV around slowly (to search for gate again)
                auto reply_msg = triton_interfaces::msg::Waypoint(); // Create a new waypoint message
                reply_msg.pose = current_pose_; // Use the current pose as the waypoint pose

                // Extract the current pose's orientation as a quaternion
                tf2::Quaternion current_pose_q(
                    current_pose_.orientation.x,
                    current_pose_.orientation.y,
                    current_pose_.orientation.z,
                    current_pose_.orientation.w);
                tf2::Matrix3x3 current_pose_q_m(current_pose_q); // Convert quaternion to a rotation matrix
                double current_pose_roll, current_pose_pitch, current_pose_yaw; // Variables to store roll, pitch, yaw
                current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw); // Extract roll, pitch, yaw

                // Set some small yaw offset to rotate slowly
                tf2::Quaternion tf2_quat_dest;
                tf2_quat_dest.setRPY(0.001, 0.001, current_pose_yaw - 0.50); // Adjust yaw for a slow rotation
                reply_msg.pose.orientation.x = tf2_quat_dest.x(); // Set the waypoint orientation (x component)
                reply_msg.pose.orientation.y = tf2_quat_dest.y(); // Set the waypoint orientation (y component)
                reply_msg.pose.orientation.z = tf2_quat_dest.z(); // Set the waypoint orientation (z component)
                reply_msg.pose.orientation.w = tf2_quat_dest.w(); // Set the waypoint orientation (w component)

                // Set small forward movement distance
                tf2::Quaternion tf2_quat_distance;
                tf2_quat_distance.setRPY(0.05, 0.05, 0.1); // Add small positional offset

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
        }
        else if (type_ == TRAJ_BUOY) {
            // RCLCPP_INFO(this->get_logger(), "Attempting buoy maneuver in TRAJ_BUOY mode.");
            switch (buoy_state_)
            {
                case BUOY_APPROACH:  // Approach the buoy
                    approach_buoy();
                    
                    // Calculate distance to buoy
                    distance_to_buoy = calculate_distance_to_buoy();
                    RCLCPP_INFO(this->get_logger(), "Buoy Pose: x=%f, y=%f, z=%f",
                            destination_pose_.position.x, destination_pose_.position.y, destination_pose_.position.z);
                    RCLCPP_INFO(this->get_logger(), "Current Pose: x=%f, y=%f, z=%f",
                        current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
                    
                    RCLCPP_INFO(this->get_logger(), "Distance to buoy is estimated at %f meters.", distance_to_buoy);


                    // Check if close enough to initiate rotation
                    
                    if (distance_to_buoy + 2  <= radius_for_rotation) // when within radius_for_rotation meter(s) of buoy, start rotation process
                    {
                        buoy_state_ = BUOY_ROTATE;
                        RCLCPP_INFO(this->get_logger(), "Approached the buoy. Proceeding to rotate around.");
                        
                        destination_achieved_ = false; // Reset destination achieved
                        

                        buoy_global_position_ = get_buoy_global_position(); // gets wrong position

                        RCLCPP_INFO(this->get_logger(), "Buoy Global Position: (%f, %f, %f)",
                        buoy_global_position_.x(),
                        buoy_global_position_.y(),
                        buoy_global_position_.z());
                    }
                    
                    break;
                case BUOY_ROTATE:  // Rotate around the buoy    
                    rotate_around_buoy();
                    buoy_state_ = BUOY_RETURN;
                    break;
                case BUOY_RETURN:  // Aim back at starting position
                    if (!destination_achieved_) {
                        aim_back_at_start(); // continuously sets waypoints towards the starting location
                    }
                    else {
                        RCLCPP_INFO(this->get_logger(), "Completed buoy maneuver. Returning to TRAJ_START mode.");
                        type_ = TRAJ_START;
                        destination_achieved_ = false;
                        buoy_state_ = BUOY_UNINITIALIZED;
                    }
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "In invalid buoy state");
                    break;
            }
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

        // Check if the detected object is a gate (change to buoy later)
        if (msg->class_id == TRAJ_GATE )
        {

            // Validate the detected buoy position to avoid extreme values
            if (abs(msg->pose.position.x) < 50 
                && abs(msg->pose.position.y) < 50
                && abs(msg->pose.position.z) < 20) {
                // Update the destination pose with the detected buoy position
                destination_pose_ = msg->pose;

                // this also implies we only know our position if we can detect a gate
                buoy_position_in_map_.x = msg->pose.position.x;
                buoy_position_in_map_.y = msg->pose.position.y;
                buoy_position_in_map_.z = msg->pose.position.z;
                
                
                if (type_ != TRAJ_BUOY || buoy_state_ == BUOY_UNINITIALIZED) {
                    // Switch to TRAJ_BUOY mode
                    type_ = TRAJ_BUOY;
                    destination_achieved_ = false;
                    buoy_state_ = BUOY_APPROACH;  // Reset the buoy state
                    starting_position_ = current_pose_.position;  // Store the starting position

                    RCLCPP_INFO(this->get_logger(), "Buoy detected! Switching to TRAJ_BUOY mode and beginning approach.");
                } 
                // else if (type_ == TRAJ_BUOY && buoy_state_ == BUOY_APPROACH) {
                //     // Update the destination pose if still approaching
                //     destination_achieved_ = false;
                // }

                // Publish the updated mode
                auto mode_msg = triton_interfaces::msg::TrajectoryType();
                mode_msg.type = type_;
                // RCLCPP_INFO(this->get_logger(), "type: %d buoy_state: %d", mode_msg.type, buoy_state_);
            }
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
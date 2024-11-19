#include "triton_controls/trajectory_generator.hpp"
using std::placeholders::_1;

namespace triton_controls
{
	// GLOBAL VARIABLES
	double rotational_theta = 0.0; // Initialize theta once

double TrajectoryGenerator::calculate_distance_to_buoy()
{
  if (destination_pose_.position.x == 0 && destination_pose_.position.y == 0 && destination_pose_.position.z == 0)
  {
	// No destination set
	return -1;
  }
  // Buoy position in map frame (assuming destination_pose_ is in the AUV's frame)
  tf2::Vector3 buoy_position_in_map(
      current_pose_.position.x + destination_pose_.position.x,
      current_pose_.position.y + destination_pose_.position.y,
      current_pose_.position.z + destination_pose_.position.z);

  // Current AUV position in map frame
  tf2::Vector3 auv_position(
      current_pose_.position.x,
      current_pose_.position.y,
      current_pose_.position.z);

  // Compute distance
  double distance = buoy_position_in_map.distance(auv_position);

  return distance;
}


void TrajectoryGenerator::approach_buoy()
{
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


void TrajectoryGenerator::rotate_around_buoy()
{
    double delta_theta = 0.1;  // Angle increment per iteration (adjust for smoothness)
    double radius = 2.0;       // Desired radius around the buoy (2 meter)

    // Update theta for counter-clockwise rotation (left around the buoy)
    rotational_theta += delta_theta;
    RCLCPP_INFO(this->get_logger(), "Rotational theta: %f", rotational_theta);

    // Reset theta after completing a full circle
    if (rotational_theta >= 2 * M_PI)
    {
        rotational_theta -= 2 * M_PI;
        destination_achieved_ = true; // Set to true if you want to proceed to the next state
        RCLCPP_INFO(this->get_logger(), "Completed circling around the buoy.");
        return;
    }

    // Buoy's position in the map frame
    // Use the stored buoy position
    double x_buoy = buoy_position_in_map_.x;
    double y_buoy = buoy_position_in_map_.y;

    // Calculate the next waypoint position along the circle
    double x_waypoint = x_buoy + radius * cos(rotational_theta);
    double y_waypoint = y_buoy + radius * sin(rotational_theta);

    // Calculate the required yaw to face the buoy
    double required_yaw = atan2(y_buoy - y_waypoint, x_buoy - x_waypoint);

    // Create the waypoint message
    auto reply_msg = triton_interfaces::msg::Waypoint();
    reply_msg.pose.position.x = x_waypoint;
    reply_msg.pose.position.y = y_waypoint;
    reply_msg.pose.position.z = current_pose_.position.z; // Maintain current depth

    // Set the orientation to face the buoy
    tf2::Quaternion tf2_quat_destination;
    tf2_quat_destination.setRPY(0, 0, required_yaw);
    reply_msg.pose.orientation = tf2::toMsg(tf2_quat_destination);

    // Set tolerances
    reply_msg.distance.position.x = 0.1; // Smaller tolerances for precision
    reply_msg.distance.position.y = 0.1;
    reply_msg.distance.position.z = 0.1;

    // Set duration and type
    // reply_msg.duration = 0.5;  // Short duration for frequent updates
    reply_msg.duration = 0;  // lets the waypoint controller handle timing.

    reply_msg.type = 0;        // STABILIZE or appropriate type

    waypoint_publisher_->publish(reply_msg);
}




void TrajectoryGenerator::aim_back_at_start()
{
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
    reply_msg.duration = 5;  // Adjust as needed
    reply_msg.type = 0;  // STABILIZE

    waypoint_publisher_->publish(reply_msg);
}


/* Constructor */
  TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions &options)
      : Node("trajectory_generator", options),
        type_(TRAJ_START),
        destination_achieved_(true),
        buoy_state_(-1)
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

  void TrajectoryGenerator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    current_pose_ = msg->pose.pose;
    auto mode_msg = triton_interfaces::msg::TrajectoryType();
    mode_msg.type = type_;
    current_mode_publisher_->publish(mode_msg);

    if (type_ == TRAJ_START) 
    {
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
    else if (type_ == TRAJ_GATE) // Check if the mode is TRAJ_GATE
    {
      // TODO: Generate trajectory logic for navigating through a gate
      if (!destination_achieved_) // If the destination hasn't been reached
      {
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
    else if (type_ == TRAJ_BUOY)
    {
        switch (buoy_state_)
        {
            case 0:  // Approach the buoy
                if (!destination_achieved_)
                {
                    approach_buoy();
					
					// Calculate distance to buoy
					double distance_to_buoy = calculate_distance_to_buoy();
					RCLCPP_INFO(this->get_logger(), "Distance to buoy is estimated at %f meters.", distance_to_buoy);


					// Check if close enough to initiate rotation
					if (distance_to_buoy <= 1.0) // when within 1 meter of buoy, start rotation process
					{
						destination_achieved_ = true;
						RCLCPP_INFO(this->get_logger(), "Reached proximity to buoy. Proceeding to rotate around.");
					}
                }
                else
                {
                    destination_achieved_ = false;
                    buoy_state_ = 1;
                    RCLCPP_INFO(this->get_logger(), "Approached the buoy. Proceeding to rotate around.");
					
                }
                break;
            case 1:  // Rotate around the buoy
                if (!destination_achieved_)
                {
					
                    rotate_around_buoy();
                }
                else
                {
                    destination_achieved_ = false;
                    buoy_state_ = 2;
                    RCLCPP_INFO(this->get_logger(), "Finished rotating around buoy. Proceeding to aim back at starting position.");
                }
                break;
            case 2:  // Aim back at starting position
                if (!destination_achieved_)
                {
                    aim_back_at_start();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Completed buoy maneuver. Returning to TRAJ_START mode.");
                    type_ = TRAJ_START;
                    destination_achieved_ = true;
                    buoy_state_ = 0;
                }
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "In invalid buoy state");
                break;
        }
    }
}



	void TrajectoryGenerator::type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg)
  {

    type_ = msg->type;
    destination_achieved_ = false;

    RCLCPP_INFO(this->get_logger(), "Trajectory Type updated. ");

  }

  void TrajectoryGenerator::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg)
  {
      /**
       * Below is the buoy_callback function implementation that currently lives in gate_callback cause I'm treating a gate like a buoy
       */

      // Check if the detected object is a gate
    if (msg->class_id == TRAJ_GATE)
    {
        // Calculate and store buoy's position in the map frame
        buoy_position_in_map_.x = current_pose_.position.x + msg->pose.position.x;
        buoy_position_in_map_.y = current_pose_.position.y + msg->pose.position.y;
        buoy_position_in_map_.z = current_pose_.position.z + msg->pose.position.z;

        
        // Validate the detected gate position to avoid extreme values
        if (abs(msg->pose.position.x) < 50 
            && abs(msg->pose.position.y) < 50
            && abs(msg->pose.position.z) < 20)
        {
            if (type_ != TRAJ_BUOY)  // Check if not already rotating around the buoy
            {
                // Update the destination pose with the detected buoy position
                destination_pose_ = msg->pose;

                

                // Switch to TRAJ_BUOY mode
                type_ = TRAJ_BUOY;
                destination_achieved_ = false;
                buoy_state_ = 0;  // Reset the buoy state
                starting_position_ = current_pose_.position;  // Store the starting position

                RCLCPP_INFO(this->get_logger(), "Buoy detected! Switching to TRAJ_BUOY mode.");

                // Publish the updated mode
                auto mode_msg = triton_interfaces::msg::TrajectoryType();
                mode_msg.type = type_;
                current_mode_publisher_->publish(mode_msg);
            }
            else
            {
                // If already rotating around the buoy, ignore new detections
                RCLCPP_INFO(this->get_logger(), "Currently rotating around buoy; ignoring new detections.");
            }
        }
    }

      /**
       * Below is the gate_callback function
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


  void TrajectoryGenerator::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {

    if (msg->success)
    {
      destination_achieved_ = true;

      if (type_ == TRAJ_BUOY) {
        RCLCPP_INFO(this->get_logger(), "Waypoint reached in TRAJ_BUOY mode.");
      } else if (type_ == TRAJ_GATE) {
        RCLCPP_INFO(this->get_logger(), "Waypoint reached in TRAJ_GATE mode.");
      }
    }

  }

} // namespace triton_controls

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::TrajectoryGenerator>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){} // during testing sometimes throws error
  return 0;
}


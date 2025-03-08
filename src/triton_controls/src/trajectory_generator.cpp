#include "triton_controls/trajectory_generator.hpp"
using std::placeholders::_1;

namespace triton_controls
{

  double radius_for_rotation = 3.0; // the destired radius of rotation (doubles as approach distance)

  TrajectoryGenerator::TrajectoryGenerator(const rclcpp::NodeOptions &options)
      : Node("trajectory_generator", options),
        type_(TRAJ_GATE),
        buoy_state_(BUOY_UNINITIALIZED),
        destination_achieved_(true),
        starting_position_set_(false) // note: this should be false, but in the interest of time, 
        // this is set to true as a way to make the AUV turn around slowly as if it is in TRAJ_START
        // mode even though it is in TRAJ_GATE mode.
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

  void TrajectoryGenerator::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    current_pose_ = msg->pose.pose;
    if (type_ == TRAJ_START) 
    {
      // Turn the AUV around slowly (to search for gate)
      auto reply_msg = triton_interfaces::msg::Waypoint();
      reply_msg.pose = msg->pose.pose;

      tf2::Quaternion current_pose_q(
        current_pose_.orientation.x,
        current_pose_.orientation.y,
        current_pose_.orientation.z,
        current_pose_.orientation.w);
      tf2::Matrix3x3 current_pose_q_m(current_pose_q);
      double current_pose_roll, current_pose_pitch, current_pose_yaw;
      current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);

      // Set some small yaw offset
      tf2::Quaternion tf2_quat_dest;
      tf2_quat_dest.setRPY(0.001, 0.001, current_pose_yaw -0.50);
      reply_msg.pose.orientation.x = tf2_quat_dest.x();
      reply_msg.pose.orientation.y = tf2_quat_dest.y();
      reply_msg.pose.orientation.z = tf2_quat_dest.z();
      reply_msg.pose.orientation.w = tf2_quat_dest.w();

      // Set some small distance
      tf2::Quaternion tf2_quat_distance;
      tf2_quat_distance.setRPY(0.05, 0.05, 0.1);

      reply_msg.distance.position.x = 0.2;
      reply_msg.distance.position.y = 0.2;
      reply_msg.distance.position.z = 0.2;
      reply_msg.distance.orientation.x = tf2_quat_distance.x();
      reply_msg.distance.orientation.y = tf2_quat_distance.y();
      reply_msg.distance.orientation.z = tf2_quat_distance.z();
      reply_msg.distance.orientation.w = tf2_quat_distance.w();
      reply_msg.duration = 2;
      reply_msg.type = 0; // STABILIZE

      waypoint_publisher_->publish(reply_msg);
    }
    else if (type_ == TRAJ_GATE)
    {
      // TOOD: generate trajectory
      if (!destination_achieved_)
      {

        auto reply_msg = triton_interfaces::msg::Waypoint();

        // reply_msg is sent to the Waypoint marker. It is in the map frame. 
        // destination_pose_ is from the gate detector. It is in the base frame. 
        // current_pose_ is also in the map frame. 

        // If the gate is not in front (on the left or right in the image), 
        // turn towards it. 
        // TODO: optimize, check math
        double required_yaw = std::atan(destination_pose_.position.y/destination_pose_.position.x);
        // std::cout << "yaw " << required_yaw << std::endl;
        // destination_pose_.position.y is in meters, should be in the single digits
        // double required_yaw = std::max(-1.57, std::min(1.57, destination_pose_.position.y/1.0 * 1.57));

        tf2::Quaternion current_pose_q(
          current_pose_.orientation.x,
          current_pose_.orientation.y,
          current_pose_.orientation.z,
          current_pose_.orientation.w);
        tf2::Matrix3x3 current_pose_q_m(current_pose_q);
        double current_pose_roll, current_pose_pitch, current_pose_yaw;
        current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);

        tf2::Quaternion tf2_quat_destination;
        tf2_quat_destination.setRPY(current_pose_roll, current_pose_pitch, current_pose_yaw + required_yaw);
        reply_msg.pose.orientation.x = tf2_quat_destination.x();
        reply_msg.pose.orientation.y = tf2_quat_destination.y();
        reply_msg.pose.orientation.z = tf2_quat_destination.z();
        reply_msg.pose.orientation.w = tf2_quat_destination.w();

        // Forward component (assuming AUV is upright, no change in z)
        // Calculate the distance to gate if the AUV faces it squarely
        double distance_x = std::sqrt(std::pow(destination_pose_.position.y,2) + std::pow(destination_pose_.position.x,2));
        // Calculate the point this far away in front of the AUV in the map frame
        tf2::Quaternion current_q;
        tf2::Vector3 dest_v;
        dest_v.setX(distance_x + 1); // Since it is a passthrough waypoint anyway
        dest_v.setY(0);
        dest_v.setZ(0);
        tf2::fromMsg(current_pose_.orientation, current_q); 
        // current_q[3] = -current_q[3]; // Invert quaternion
        tf2::Vector3 targetForward = tf2::quatRotate(current_q, dest_v);
        reply_msg.pose.position.x = current_pose_.position.x + targetForward.getX();
        reply_msg.pose.position.y = current_pose_.position.y + targetForward.getY();

        // Z
        reply_msg.pose.position.z = current_pose_.position.z + destination_pose_.position.z;


        tf2::Quaternion tf2_quat_distance;
        tf2_quat_distance.setRPY(1.57, 1.57, 1.57); 

        // Assume that the gate aligns with the y-axis, 
        // i.e. a straight path on the x-axis goes through it
        reply_msg.distance.position.x = 0.5; 
        reply_msg.distance.position.y = 4.0; // Assume 2 meters wide gate
        reply_msg.distance.position.z = 2.0; // Assume 1 meter tall gate
        reply_msg.distance.orientation.x = tf2_quat_distance.x();
        reply_msg.distance.orientation.y = tf2_quat_distance.y();
        reply_msg.distance.orientation.z = tf2_quat_distance.z();
        reply_msg.distance.orientation.w = tf2_quat_distance.w();
        reply_msg.type = 1;  // PASSTHROUGH

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
        
      // else  // reached the gate, (assuming we are right at the gate)
      // {
        // RCLCPP_INFO(this->get_logger(), "Reached the gates");

        //       // Stop residual movement and stabilize
        // auto stop_msg = triton_interfaces::msg::Waypoint();
        // stop_msg.pose.position = current_pose_.position; // Stay in place
        // stop_msg.pose.orientation = current_pose_.orientation; // Keep current orientation

        // stop_msg.distance.position.x = 0.1; // Small tolerance
        // stop_msg.distance.position.y = 0.1;
        // stop_msg.distance.position.z = 0.1;

        // waypoint_publisher_->publish(stop_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(1000)); // time to stabilize

        // type_ = TRAJ_BUOY;
        // buoy_state_ = BUOY_APPROACH;

      // }
    }
    else if (type_ == TRAJ_BUOY) {
            // RCLCPP_INFO(this->get_logger(), "Attempting buoy maneuver in TRAJ_BUOY mode.");
            switch (buoy_state_)
            {
                case BUOY_APPROACH:  // Approach the buoy, prepare for rotation
                    // approach_buoy();

                    // TRAVEL 2 meters (hard coded)
                    
                    // Calculate distance to buoy
                    // distance_to_buoy = calculate_distance_to_buoy();

                    RCLCPP_INFO(this->get_logger(), "Approached the buoy. Proceeding to rotate around.");
                    
                    destination_achieved_ = false; // Reset destination achieved
                    
                    buoy_global_position_ = get_buoy_global_position();
                     // gets hardcoded position based on distance from gate

                    RCLCPP_INFO(this->get_logger(), "Buoy Global Position: (%f, %f, %f)",
                    buoy_global_position_.x(),
                    buoy_global_position_.y(),
                    buoy_global_position_.z());
                    buoy_state_ = BUOY_ROTATE;

                    
                    break;
                case BUOY_ROTATE:  // Rotate around the buoy    
                    rotate_around_buoy();
                    RCLCPP_INFO(this->get_logger(), "Rotation");
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

    // THIS IS HARDCODED, WE NEED A BUOY DETECTOR FROM CV TEAM s
    tf2::Vector3 TrajectoryGenerator::get_buoy_global_position() {
        // 1. Convert the AUV's current quaternion to a tf2::Quaternion
        tf2::Quaternion current_q;
        tf2::fromMsg(current_pose_.orientation, current_q);

        // 2. The buoy offset in the AUV’s local frame
        // TODO need buoy detector to caluclate offset accurately, for now hard coded to 2 meters

        // 3. Rotate hardcoded two meter offset from the gate to get the buoy offset in the GLOBAL frame
        tf2::Vector3 global_offset = {2, 0, 0};
        global_offset = tf2::quatRotate(current_q, global_offset);

        // 4. Now add that global offset to the AUV’s current global position
        tf2::Vector3 buoy_global_pos(
            current_pose_.position.x + global_offset.x(),
            current_pose_.position.y + global_offset.y(),
            current_pose_.position.z + global_offset.z()
        );

        return buoy_global_pos;
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
      }
      // After completing the loop, return to start position (change to buoy return state)
      buoy_state_ = BUOY_RETURN; // Transition to return state
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

  void TrajectoryGenerator::type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg)
  {
    // if (!(buoy_state_ == BUOY_ROTATE) && !(type_ == TRAJ_BUOY)) {
            type_ = msg->type;
            destination_achieved_ = false;

            RCLCPP_INFO(this->get_logger(), "Trajectory Type updated. ");

  }

  void TrajectoryGenerator::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg)
  {


    if (msg->class_id == type_ && type_ == TRAJ_START)
    {
      // Catch extreme cases when the gate detector did not detect properly
      if (abs(msg->pose.position.x) < 50 
          && abs(msg->pose.position.y) < 50
          && abs(msg->pose.position.z) < 20)
      {
        destination_achieved_ = false;
        destination_pose_ = msg->pose;

        if (!starting_position_set_) {
            // buoy_state_ = BUOY_APPROACH;                 
            starting_position_ = current_pose_.position;  // Store the starting position  
            starting_position_set_ = true;                 
        }

      }
    }

  }

  void TrajectoryGenerator::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {
    if (msg->success)
    {
      destination_achieved_ = true;
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


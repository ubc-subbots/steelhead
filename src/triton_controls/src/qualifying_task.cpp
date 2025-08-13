#include "triton_controls/qualifying_task.hpp"
using std::placeholders::_1;
#include <iostream>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::milliseconds, seconds, etc.


namespace triton_controls
{
    bool _start = true;

    float prevY;
    float prevZ;

    int yCycles;
    int zCycles;

    const int numCycles = 3000;

    geometry_msgs::msg::Wrench outputForce;

  QualifyingTask::QualifyingTask(const rclcpp::NodeOptions &options)
      : Node("qualifying_task", options),
        type_(TRAJ_GATE),
        destination_achieved_(true) // note: this should be false, but in the interest of time, 
        // this is set to true as a way to make the AUV turn around slowly as if it is in TRAJ_START
        // mode even though it is in TRAJ_GATE mode.
  {

    outputForce = geometry_msgs::msg::Wrench();
    int yCycles = numCycles;
    int zCycles = numCycles;
    // ros2 topic pub /triton/controls/input_forces geometry_msgs/msg/Wrench "{force: {x: 15.0, y: 0, z: 0}}"

    waypoint_publisher_ = this->create_publisher<triton_interfaces::msg::Waypoint>("/triton/controls/waypoint_marker/set", 10);

    input_forces_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/triton/controls/input_forces", 10);

    state_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/triton/controls/ukf/odometry/filtered", 10, std::bind(&QualifyingTask::state_callback, this, _1));

    type_subscription_ = this->create_subscription<triton_interfaces::msg::TrajectoryType>(
        "/triton/controls/qualifying_task/set_type", 10, std::bind(&QualifyingTask::type_callback, this, _1));

    gate_subscription_ = this->create_subscription<triton_interfaces::msg::ObjectOffset>(
        "/triton/gate/detector/gate_pose", 10, std::bind(&QualifyingTask::gate_callback, this, _1));

    // this->declare_parameter("start_turning_factor", start_turning_factor_);
    // this->get_parameter("start_turning_factor", start_turning_factor_);
    

    RCLCPP_INFO(this->get_logger(), "Qualifying task successfully started!");
  }

  void QualifyingTask::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    current_pose_ = msg->pose.pose;
    if (_start == true)
    {
      // TOOD: generate trajectory
      if (!destination_achieved_)
      {

        // triton_interfaces::msg::ObjectOffset gateMsg;
        // rclcpp::MessageInfo msgInfo;

        // if (gate_subscription_->take(gateMsg, msgInfo)) {
        //   if (gateMsg.pose.position.y > prevY) {
        //     outputForce.torque.z = -1;
        //      std::cout << "turning right" << std::endl;
        //   } else {
        //     outputForce.torque.z = 1;
        //      std::cout << "turing left" << std::endl;
        //   }

        //   if (gateMsg.pose.position.z > prevZ) {
        //     outputForce.force.z = 3;
        //     std::cout << "going up" << std::endl;
        //   } else {
        //     outputForce.force.z = -3;
        //     std::cout << "going down" << std::endl;
        //   }
        // }

        // outputForce.force.x = 15;

        // input_forces_publisher_->publish(outputForce);
      }
      else  // This is based the TRAJ_START branch above
      // because there is no time for making the gate-detected-confirmer
      // so QualifyingTask will start with type == TRAJ_GATE and 
      // turn around when the detection makes no sense. 
      {
        // Turn the AUV around slowly (to search for gate)
        auto reply_msg = geometry_msgs::msg::Wrench();

        reply_msg.torque.z = 5;

        input_forces_publisher_->publish(reply_msg);
        }
    } else {
         // Turn the AUV around slowly (to search for gate)
        auto reply_msg = geometry_msgs::msg::Wrench();
        
        input_forces_publisher_->publish(reply_msg);
    }

  }

  void QualifyingTask::type_callback(const triton_interfaces::msg::TrajectoryType::SharedPtr msg)
  {

    type_ = msg->type;
    destination_achieved_ = false;

    RCLCPP_INFO(this->get_logger(), "Trajectory Type updated. ");

  }

  void QualifyingTask::gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg)
  {

    if (msg->class_id == type_ && type_ == TRAJ_GATE)
    {
      // Catch extreme cases when the gate detector did not detect properly
      if (abs(msg->pose.position.x) < 50 
          && abs(msg->pose.position.y) < 50
          && abs(msg->pose.position.z) < 20)
      {
        destination_achieved_ = false;
        destination_pose_ = msg->pose;
      }
    }

    if (msg->pose.position.y > prevY) {
        if (yCycles == 0) {
          outputForce.torque.z = -1;
          std::cout << "turning right" << std::endl;
          yCycles = numCycles;
        } else {
          yCycles--;
        }
         
      } else {
        if (yCycles == 0) {
          outputForce.torque.z = 1;
          std::cout << "turing left" << std::endl;
          yCycles = numCycles;
        } else {
          yCycles--;
        }
      }

      if (zCycles == 0) {
        if (msg->pose.position.z > prevZ) {
          outputForce.force.z = 3;
          std::cout << "going up" << std::endl;
        } else {
          outputForce.force.z = -3;
          std::cout << "going down" << std::endl;
        }
        zCycles = numCycles;
      } else 
      {
        zCycles--;
      }
    
    prevY = msg->pose.position.y;
    prevZ = msg->pose.position.z;

    if (_start == true && !destination_achieved_) {
      outputForce.force.x = 15;
    }

    input_forces_publisher_->publish(outputForce);

    

    // std::cout << "x:" << msg->pose.position.x << std::endl;
    // std::cout << "y:" << msg->pose.position.y << std::endl;
    // std::cout << "z:" << msg->pose.position.z << std::endl;

  }

  void QualifyingTask::waypoint_callback(const triton_interfaces::msg::Waypoint::SharedPtr msg)
  {

    if (msg->success)
    {
      destination_achieved_ = true;
      _start = false;
    }

  }

} // namespace triton_controls

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<triton_controls::QualifyingTask>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const&){} // during testing sometimes throws error
  return 0;
}

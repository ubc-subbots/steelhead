#include "steelhead_gazebo/actuators_command_simulation.hpp"
using std::placeholders::_1;

namespace steelhead_gazebo
{
  ActuatorsCommandSimulation::ActuatorsCommandSimulation(const rclcpp::NodeOptions & options)
  : Node("actuators_command", options)
  {
      service_ = this->create_service<steelhead_interfaces::srv::ActuatorsCommand>(
        "actuators_command", 
        std::bind(&ActuatorsCommandSimulation::handleRequest, this,
        std::placeholders::_1,
        std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "Actuators simulation server successfully started!");

      // map actions defined in actuators_config.yaml to their functions TODO define custom functions in config rather than hardcoding
      action_map_["torpedo"] = std::bind(&ActuatorsCommandSimulation::handleTorpedo, this, std::placeholders::_1);
      action_map_["claw"] = std::bind(&ActuatorsCommandSimulation::handleClaw, this, std::placeholders::_1);

      torpedo_fire_pub_ = this->create_publisher<std_msgs::msg::Empty>("/steelhead/torpedo/fire", 10);
  }

  ActuatorsCommandSimulation::~ActuatorsCommandSimulation() {
  }

  void ActuatorsCommandSimulation::handleRequest(const std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Request> request,
          std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response>      response) {
            auto it = action_map_.find(request->input);

            if (it != action_map_.end()) {
              it->second(response);
            } else {
              RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", request->input.c_str());
              response->succeeded = false;
            }
          }

  void ActuatorsCommandSimulation::handleTorpedo(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Firing Torpedoes");

      std_msgs::msg::Empty msg;
      torpedo_fire_pub_->publish(msg);

      response->succeeded = true;
  }

  void ActuatorsCommandSimulation::handleClaw(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Toggling Claw");
      // TODO simulate a claw in gazebo
      response->succeeded = true;
  }
} // namespace steelhead_gazebo

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto actuators_command_node = std::make_shared<steelhead_gazebo::ActuatorsCommandSimulation>(
        rclcpp::NodeOptions());

    rclcpp::spin(actuators_command_node);
    
    rclcpp::shutdown();
    return 0;
}

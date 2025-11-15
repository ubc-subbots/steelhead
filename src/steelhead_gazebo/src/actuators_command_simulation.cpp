#include "steelhead_gazebo/actuators_command_simulation.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <cstdlib>
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

      // connect to gazebo spawning service
      spawner_node_ = rclcpp::Node::make_shared("sdf_spawner_node");
      spawner_client_ = spawner_node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
      while (!spawner_client_->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(spawner_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return;
          }
      }
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
      
      /////////////////////////////////////////////////////FOR DEMO PURPOSES////////////////////////////////////////////////////
      // load SDF content from a file
      auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
      std::ifstream sdf_file(ament_index_cpp::get_package_share_directory("steelhead_gazebo") + "/gazebo/models/steelhead_thruster/model.sdf");
      std::string sdf_content((std::istreambuf_iterator<char>(sdf_file)), std::istreambuf_iterator<char>());
      request->xml = sdf_content;
      request->name = rand() % 100;
      request->robot_namespace = "steelhead_gazebo"; 
      request->initial_pose.position.x = 0.0;
      request->initial_pose.position.y = 0.0;
      request->initial_pose.position.z = rand() % 2 - 4.0;

      // send request
      auto result = spawner_client_->async_send_request(request);
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
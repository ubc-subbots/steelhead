#include "steelhead_gazebo/actuators_command_simulation.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include <ament_index_cpp/get_package_share_path.hpp>
#include <fstream>
#include <cstdlib>
using std::placeholders::_1;

namespace steelhead_gazebo
{
  using ActuatorCommand = steelhead_interfaces::srv::ActuatorsCommand::Request;

  ActuatorsCommandSimulation::ActuatorsCommandSimulation(const rclcpp::NodeOptions & options)
  : Node("actuators_command", options)
  {
      service_ = this->create_service<steelhead_interfaces::srv::ActuatorsCommand>(
        "actuators_command", 
        std::bind(&ActuatorsCommandSimulation::handleRequest, this,
        std::placeholders::_1,
        std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "Actuators simulation server successfully started!");

      action_map_[ActuatorCommand::FIRE_LEFT_TORPEDO] = std::bind(&ActuatorsCommandSimulation::handleTorpedo, this, std::placeholders::_1);
      action_map_[ActuatorCommand::FIRE_RIGHT_TORPEDO] = std::bind(&ActuatorsCommandSimulation::handleTorpedo, this, std::placeholders::_1);
      action_map_[ActuatorCommand::OPEN_CLAW] = std::bind(&ActuatorsCommandSimulation::handleClaw, this, std::placeholders::_1);
      action_map_[ActuatorCommand::CLOSE_CLAW] = std::bind([](auto response){response->succeeded = true;}, std::placeholders::_1);

      // connect to gazebo spawning service (assuming default world name 'world', may need param)
      spawner_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/world/create");
      while (!spawner_client_->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
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
      
      auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
      std::ifstream sdf_file(ament_index_cpp::get_package_share_path("steelhead_gazebo").string() + "/gazebo/models/steelhead_torpedo/model.sdf");
      std::string sdf_content((std::istreambuf_iterator<char>(sdf_file)), std::istreambuf_iterator<char>());
      
      request->entity_factory.name = "torpedo_" + std::to_string(this->now().nanoseconds());
      request->entity_factory.sdf = sdf_content;
      // In Gazebo Sim, reference frame requires careful setup or specifying pose in world frame.
      request->entity_factory.pose.position.x = 0.2;
      request->entity_factory.pose.position.y = 0.0;
      request->entity_factory.pose.position.z = -0.2;

      // send request
      auto result = spawner_client_->async_send_request(request);
      response->succeeded = true;
  }

  void ActuatorsCommandSimulation::handleClaw(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Toggling Claw / Dropping Marker");

      auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
      std::ifstream sdf_file(ament_index_cpp::get_package_share_path("steelhead_gazebo").string() + "/gazebo/models/steelhead_dropper_marker/model.sdf");
      std::string sdf_content((std::istreambuf_iterator<char>(sdf_file)), std::istreambuf_iterator<char>());
      
      request->entity_factory.name = "dropper_" + std::to_string(this->now().nanoseconds());
      request->entity_factory.sdf = sdf_content;
      request->entity_factory.pose.position.x = 0.3;
      request->entity_factory.pose.position.y = 0.0;
      request->entity_factory.pose.position.z = -0.2;

      // send request
      auto result = spawner_client_->async_send_request(request);
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

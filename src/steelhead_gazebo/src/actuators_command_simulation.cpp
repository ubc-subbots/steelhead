#include "steelhead_gazebo/actuators_command_simulation.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
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

      // connect to gazebo spawning service
      spawner_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
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
      
      // load SDF content from a file
      auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
      std::ifstream sdf_file(ament_index_cpp::get_package_share_directory("steelhead_gazebo") + "/gazebo/models/steelhead_torpedo/model.sdf");
      std::string sdf_content((std::istreambuf_iterator<char>(sdf_file)), std::istreambuf_iterator<char>());
      request->name = "torpedo_" + std::to_string(this->now().nanoseconds());
      request->xml = sdf_content;
      request->robot_namespace = "steelhead_gazebo"; 
      request->reference_frame = "steelhead_auv";
      request->initial_pose.position.x = 0.2;
      request->initial_pose.position.y = 0.0;
      request->initial_pose.position.z = -0.2;

      // send request
      auto result = spawner_client_->async_send_request(request);
      response->succeeded = true;
  }

  // right now, we are only using the claw for the dropper task. !TODO add claw functionality
  void ActuatorsCommandSimulation::handleClaw(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Toggling Claw / Dropping Marker");

      // load SDF content from a file
      auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
      std::ifstream sdf_file(ament_index_cpp::get_package_share_directory("steelhead_gazebo") + "/gazebo/models/steelhead_dropper_marker/model.sdf");
      std::string sdf_content((std::istreambuf_iterator<char>(sdf_file)), std::istreambuf_iterator<char>());
      request->name = "dropper_" + std::to_string(this->now().nanoseconds());
      request->xml = sdf_content;
      request->robot_namespace = "steelhead_gazebo"; 
      request->reference_frame = "steelhead_auv";
      request->initial_pose.position.x = 0.3;
      request->initial_pose.position.y = 0.0;
      request->initial_pose.position.z = -0.2;

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
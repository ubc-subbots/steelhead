#ifndef STEELHEAD_CONTROL__ACTUATORS_COMMAND_SIMULATION
#include "std_msgs/msg/u_int32.hpp"
#include "steelhead_interfaces/srv/actuators_command.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include "rclcpp/rclcpp.hpp"
namespace steelhead_gazebo
{      

    class ActuatorsCommandSimulation : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * @param options ros2 node options.
         */
        explicit ActuatorsCommandSimulation(const rclcpp::NodeOptions & options);
        ~ActuatorsCommandSimulation();

    private:
        std::map<std::string, std::function<void(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response>)>> action_map_;
        rclcpp::Service<steelhead_interfaces::srv::ActuatorsCommand>::SharedPtr service_;
        
        rclcpp::Node::SharedPtr spawner_node_;
        std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SpawnEntity>> spawner_client_;

        void handleRequest(const std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Request> request,
          std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response>      response);
        void handleClaw(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response);
        void handleTorpedo(std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response> response);
    };
    
} // namespace steelhead_gazebo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_gazebo::ActuatorsCommandSimulation)

#endif  //STEELHEAD_CONTROL__ACTUATORS_COMMAND_SIMULATION

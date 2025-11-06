#ifndef STEELHEAD_CONTROL__ACTUATORS_COMMAND
#include "std_msgs/msg/u_int32.hpp"
#include "steelhead_interfaces/srv/actuators_command.hpp"

#include "rclcpp/rclcpp.hpp"
namespace steelhead_controls
{      

    class ActuatorsCommand : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * @param options ros2 node options.
         */
        explicit ActuatorsCommand(const rclcpp::NodeOptions & options);
        ~ActuatorsCommand();

    private:
        int fd_;
        std::map<std::string,int> nameToPin;
        rclcpp::Service<steelhead_interfaces::srv::ActuatorsCommand>::SharedPtr service_;

        void sendOverSerial(const std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Request> request,
          std::shared_ptr<steelhead_interfaces::srv::ActuatorsCommand::Response>      response);
    };
    
} // namespace steelhead_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_controls::ActuatorsCommand)

#endif  //STEELHEAD_CONTROL__ACTUATORS_COMMAND

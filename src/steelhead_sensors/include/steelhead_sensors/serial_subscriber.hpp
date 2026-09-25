#ifndef STEELHEAD_CONTROL__SERIAL_SUBSCRIBER 
#include "std_msgs/msg/u_int32.hpp"

#include "rclcpp/rclcpp.hpp"
namespace steelhead_sensors
{      

    class SerialSubscriber : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * @param options ros2 node options.
         */
        explicit SerialSubscriber(const rclcpp::NodeOptions & options);
        ~SerialSubscriber();

    private:
        void controlCallback(const std_msgs::msg::UInt32::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr thruster_sub_; 
        int fd_;
    };
    
} // namespace steelhead_sensors

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_sensors::SerialSubscriber)

#endif  //STEELHEAD_CONTROL__SERIAL_SUBSCRIBER

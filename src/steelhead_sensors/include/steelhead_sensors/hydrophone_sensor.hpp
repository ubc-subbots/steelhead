#ifndef STEELHEAD_EXAMPLE__HYDROPHONE_SENSOR
#define STEELHEAD_EXAMPLE__HYDROPHONE_SENSOR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "gz/msgs/pose_v.pb.h"
#include "gz/transport/Node.hh"

#include <steelhead_interfaces/msg/hydrophone_feedback.hpp>
#include <steelhead_interfaces/msg/pinger_angle.hpp>

#include <unordered_map>
#include <string>



namespace steelhead_sensors
{      

    class HydrophoneSensor : public rclcpp::Node
    {

    public:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        explicit HydrophoneSensor(const rclcpp::NodeOptions & options);

    private:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        
        rclcpp::Publisher<steelhead_interfaces::msg::HydrophoneFeedback>::SharedPtr publisher_;  
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_; 
        gz::transport::Node gz_node_;

        // Robot position
        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_yaw_ = 0.0;

        // Pinger position
        double pinger_x_ = 0.0;
        double pinger_y_ = 0.0;

        // Track whether we've received data
        bool robot_received_ = false;
        bool pinger_received_ = false;

        struct PingerPosition {
            double x;
            double y;
        };
        std::unordered_map<std::string, PingerPosition> pingers_;

        void robot_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pinger_callback(const gz::msgs::Pose_V & msg);

        void calculate_relative_position();

    };
    
} // namespace steelhead_sensors

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_sensors::HydrophoneSensor)

#endif  //STEELHEAD_EXAMPLE__HYDROPHONE_SENSOR

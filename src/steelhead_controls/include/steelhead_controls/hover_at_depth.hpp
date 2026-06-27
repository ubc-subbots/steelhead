#ifndef STEELHEAD_CONTROL__HOVER_AT_DEPTH
#define STEELHEAD_CONTROL__HOVER_AT_DEPTH

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "steelhead_interfaces/msg/pressure_sensor.hpp"
#include <math.h>

namespace steelhead_controls
{

    class HoverAtDepth : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Creates the hover script with inputs, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit HoverAtDepth(const rclcpp::NodeOptions & options);

    private:
        /** IMU message callback
         * 
         * Updates private variable containing IMU message and calls the input pose publish function
         * 
         * @param msg message containing imu data
         */
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        /** Pressure sensor message callback
         * 
         * Updates private variable containing depth and calls the input pose publish function
         * 
         * @param msg pressure sensor message that contains depth
         */
        void depth_callback(const steelhead_interfaces::msg::PressureSensor::SharedPtr msg);

        /** Wrench message callback
         * 
         * Updates private variable containing adjustment wrench, which will adjust the error pose before being published
         * 
         * @param msg wrench msg that contains the desired adjustments
         */
        void wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

        /** Input pose callback
         * 
         * Callback to publish error to pose for PID controller, called by either of the other callbacks when updated
         * 
         */
        void publish_error_to_target();

        float hover_depth_;
        bool hold_yaw_;

        sensor_msgs::msg::Imu::SharedPtr imu_;
        steelhead_interfaces::msg::PressureSensor::SharedPtr pressure_sensor_;
        geometry_msgs::msg::Wrench::SharedPtr adjustments_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Subscription<steelhead_interfaces::msg::PressureSensor>::SharedPtr pressure_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_subscription_;
    };

} // namespace steelhead_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_controls::HoverAtDepth)

#endif  //STEELHEAD_CONTROL__HOVER_AT_DEPTH

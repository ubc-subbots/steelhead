#ifndef STEELHEAD_TASKS__PITCH_FLIP
#define STEELHEAD_TASKS__PITCH_FLIP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace steelhead_tasks
{

    class PitchFlip : public rclcpp::Node
    {

    public:

        /** Constructor
         *
         * Declares the flip torque and flip angle parameters, sets up the
         * hover adjustment and pipeline feedback publishers and the IMU
         * subscriber.
         *
         * @param options ros2 node options.
         */
        explicit PitchFlip(const rclcpp::NodeOptions & options);

    private:

        /** IMU message callback
         *
         * Latches the starting orientation on the first message, then
         * accumulates the pitch rotated between consecutive messages. While
         * the accumulated pitch is below the flip angle (3pi/2 by default),
         * publishes a full hover adjustment with the flip torque. Once
         * reached, publishes a zero partial adjustment and reports success so
         * the hover node takes over and levels out the last quarter of the
         * flip, ending on the orientation the flip started at.
         *
         * @param msg message containing imu data
         */
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        bool done_;
        bool started_;
        double accumulated_pitch_;

        tf2::Quaternion start_orientation_;
        tf2::Quaternion prev_orientation_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::PitchFlip)

#endif  //STEELHEAD_TASKS__PITCH_FLIP

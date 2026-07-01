#ifndef STEELHEAD_TASKS__YAW_SPIN
#define STEELHEAD_TASKS__YAW_SPIN

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    /** Automated task node that spins the AUV a full yaw turn using the IMU.
     *
     * On the first IMU message the starting heading is latched. The node then
     * commands a constant yaw adjustment on the hover_at_depth adjustment
     * topic (hover_at_depth uses only the sign of torque.z, and yaw
     * adjustments override its hold_yaw parameter) while accumulating the
     * unwrapped yaw travel reported by the IMU. Once a full revolution per
     * requested turn has been swept, it trims back onto the starting heading,
     * so the heading is the same before and after the task, then publishes a
     * zero wrench and notifies the pipeline manager via a PipelineFeedback
     * message so the pipeline sequence can advance. Intended for gate style
     * points, where each 90 degree yaw change scores.
     *
     * The number of turns (sign sets the spin direction) and the final heading
     * tolerance are re-read from the node parameters every control tick,
     * because the pipeline manager applies its param file only after the
     * component is loaded; this also makes them tunable at runtime with
     * `ros2 param set`.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container.
     */
    class YawSpin : public rclcpp::Node
    {

    public:

        /** Construct the yaw spin task node.
         *
         * Declares parameters, subscribes to the IMU, sets up the hover
         * adjustment and pipeline feedback publishers, and starts the control
         * loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit YawSpin(const rclcpp::NodeOptions & options);

    private:

        /** Handle an incoming IMU message.
         *
         * Extracts the yaw from the IMU orientation, latches the starting
         * heading on the first message, and accumulates the unwrapped yaw
         * travelled since the last message so the control loop can tell how
         * far the AUV has rotated.
         *
         * @param msg the latest IMU reading.
         */
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        /** Control loop, called on a fixed timer.
         *
         * Commands a yaw adjustment in the spin direction until a full
         * revolution per requested turn has been accumulated, then commands
         * yaw back toward the starting heading. Once the heading is within
         * tolerance of the start, publishes a zero wrench and reports success
         * to the pipeline manager.
         */
        void control_loop();

        // heading (rad) when the task started, latched on the first IMU message
        double start_yaw_;
        // yaw (rad) from the most recent IMU message
        double yaw_;
        // yaw from the previous IMU message, used to unwrap the travel
        double prev_yaw_;
        // total yaw travelled since the start, unwrapped (a full turn is 2*pi)
        double accumulated_;
        // whether an IMU message has been received yet
        bool have_imu_;
        // whether success has been reported, so it is only reported once
        bool done_;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::YawSpin)

#endif  //STEELHEAD_TASKS__YAW_SPIN

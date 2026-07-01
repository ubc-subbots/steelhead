#ifndef STEELHEAD_TASKS__TIMED_WRENCH
#define STEELHEAD_TASKS__TIMED_WRENCH

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    /** Automated task node that applies a configured wrench for a configured time.
     *
     * On a fixed timer this node publishes a constant Wrench on the
     * hover_at_depth adjustment topic (force.x/y nudge position, force.z sign
     * nudges depth, torque.z sign nudges yaw). Once duration seconds have
     * elapsed it publishes a zero wrench to stop the motion and notifies the
     * pipeline manager via a PipelineFeedback message so the pipeline sequence
     * can advance. Use it for open-loop maneuvers that need no sensing, e.g.
     * gate style spins (torque_z only) or blind forward legs (force_x only).
     *
     * The wrench components and the duration are re-read from the node
     * parameters every control tick, because the pipeline manager applies its
     * param file only after the component is loaded; this also makes the
     * maneuver tunable at runtime with `ros2 param set`.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container.
     */
    class TimedWrench : public rclcpp::Node
    {

    public:

        /** Construct the timed wrench task node.
         *
         * Declares parameters, sets up the hover adjustment and pipeline
         * feedback publishers, records the start time, and starts the control
         * loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit TimedWrench(const rclcpp::NodeOptions & options);

    private:

        /** Control loop, called on a fixed timer.
         *
         * Publishes the configured wrench until duration seconds have passed
         * since the node was loaded, then publishes a zero wrench and reports
         * success to the pipeline manager.
         */
        void control_loop();

        // time the node was constructed; the duration is measured from here
        rclcpp::Time start_time_;
        // whether success has been reported, so it is only reported once
        bool done_;

        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::TimedWrench)

#endif  //STEELHEAD_TASKS__TIMED_WRENCH

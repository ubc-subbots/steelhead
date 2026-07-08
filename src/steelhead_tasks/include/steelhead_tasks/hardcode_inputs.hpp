#ifndef STEELHEAD_TASKS__HARDCODE_INPUTS
#define STEELHEAD_TASKS__HARDCODE_INPUTS

#include "rclcpp/rclcpp.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    class HardcodeInputs : public rclcpp::Node
    {

    public:

        /** Constructor
         *
         * Declares the wrench and duration parameters, sets up the hover
         * adjustment and pipeline feedback publishers and starts the
         * publishing timer.
         *
         * @param options ros2 node options.
         */
        explicit HardcodeInputs(const rclcpp::NodeOptions & options);

    private:

        /** Timer callback
         *
         * While the configured duration has not elapsed, publishes the
         * parameter-configured wrench as a partial hover adjustment so the
         * hover node keeps balancing the AUV. Once the duration elapses,
         * publishes a zero wrench and reports success to the pipeline
         * manager.
         *
         * @note The wrench and duration parameters are read every tick since
         * the pipeline manager loads the param file after the component is
         * loaded into the container.
         */
        void timer_callback();

        bool done_;
        rclcpp::Time start_time_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::HardcodeInputs)

#endif  //STEELHEAD_TASKS__HARDCODE_INPUTS

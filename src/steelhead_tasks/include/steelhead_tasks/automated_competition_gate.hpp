#ifndef STEELHEAD_TASKS__AUTOMATED_COMPETITION_GATE
#define STEELHEAD_TASKS__AUTOMATED_COMPETITION_GATE

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    /** Automated task node for passing through the competition gate.
     *
     * This node subscribes to the bounding boxes published by the object
     * recognizer, and on a fixed timer decides how the AUV should move to line
     * up with and drive through the gate. Movement commands are published as a
     * partial HoverAdjustment on the hover_at_depth adjustment topic (the
     * wrapped Wrench's force.x/y nudge position, force.z sign nudges depth,
     * torque.z sign nudges yaw). When the gate has
     * been passed the node notifies the pipeline manager via a PipelineFeedback
     * message so the pipeline sequence can advance.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container. Use this as the reference example for
     * other automated task nodes in this package.
     */
    class AutomatedCompetitionGate : public rclcpp::Node
    {

    public:

        /** Construct the automated competition gate task node.
         *
         * Declares parameters, subscribes to the detections topic, sets up the
         * hover adjustment and pipeline feedback publishers, and starts the
         * control loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit AutomatedCompetitionGate(const rclcpp::NodeOptions & options);

    private:

        /** Handle an incoming set of detections.
         *
         * Caches the most recent gate bounding box (highest confidence box whose
         * class_id matches the gate) so the control loop can act on it.
         *
         * @param msg the latest detections from the object recognizer.
         */
        void detections_callback(const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg);

        /** Handle camera calibration info.
         *
         * Reads the image width from the camera's CameraInfo (populated from the
         * camera calibration file) so the control loop knows where the image
         * centre is. The resolution does not change at runtime, so this reads the
         * width once and then resets the subscription to stop listening.
         *
         * @param msg the camera info for the front camera.
         */
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /** Control loop, called on a fixed timer.
         *
         * Computes a movement adjustment from the most recent gate detection and
         * publishes it to the hover_at_depth adjustment topic. Publishes pipeline
         * feedback once the gate is judged to have been passed.
         */
        void control_loop();

        // label published by the object recognizer
        std::string target_detection_label_;
        // horizontal size (px) of the camera image, read once from CameraInfo;
        // 0 until the first CameraInfo message arrives
        double image_width_;
        // seconds to keep driving after the target detection is lost before
        // the gate is considered passed
        double pass_timeout_;

        // most recent target box, valid only while have_gate_ is true
        steelhead_interfaces::msg::DetectionBox latest_detected_box_;
        bool have_gate_;
        // whether the target has ever been detected, latched on first sighting
        bool have_seen_target_;
        // time the target was last detected, used for the pass timeout
        rclcpp::Time last_detection_time_;

        rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::AutomatedCompetitionGate)

#endif  //STEELHEAD_TASKS__AUTOMATED_COMPETITION_GATE
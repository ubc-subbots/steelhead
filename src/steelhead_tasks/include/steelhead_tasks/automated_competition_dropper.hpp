#ifndef STEELHEAD_TASKS__AUTOMATED_COMPETITION_DROPPER
#define STEELHEAD_TASKS__AUTOMATED_COMPETITION_DROPPER

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"
#include "steelhead_interfaces/srv/actuators_command.hpp"

namespace steelhead_tasks
{

    /** Automated task node for the competition dropper (bin) task.
     *
     * This node subscribes to the bounding boxes published by the object
     * recognizer on the bottom (downward) camera, and attempts to center the
     * detection under the sub in both axes. Once the target is centered it
     * opens the claw to drop the marker.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container. 
     */
    class AutomatedCompetitionDropper : public rclcpp::Node
    {

    public:

        /** Construct the automated competition dropper task node.
         *
         * Declares parameters, subscribes to the bottom camera detections topic,
         * sets up the hover adjustment and pipeline feedback publishers, and
         * starts the control loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit AutomatedCompetitionDropper(const rclcpp::NodeOptions & options);

    private:

        /** Handle an incoming set of detections.
         *
         * Caches the highest-confidence bounding box whose label matches the
         * target so the control loop can act on it.
         *
         * @param msg the latest detections from the object recognizer.
         */
        void detections_callback(const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg);

        /** Handle camera calibration info.
         *
         * Reads the image width and height from the bottom camera's CameraInfo so
         * the control loop knows where the image centre is. The resolution does
         * not change at runtime, so this reads once and then resets the
         * subscription to stop listening.
         *
         * @param msg the camera info for the bottom camera.
         */
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /** Control loop, called on a fixed timer.
         *
         * Computes a movement adjustment from the most recent detection and
         * publishes it to the hover_at_depth adjustment topic. Drops the marker
         * and publishes pipeline feedback once the target is centered.
         */
        void control_loop();

        /** Drop the marker by opening the claw.
         *
         * Sends an asynchronous request to the actuators_command service with the
         * given command. The response is handled in a callback so the control loop
         * is never blocked (a synchronous wait would deadlock this component
         * node's single-threaded executor).
         *
         * @param input one of the ActuatorsCommand request constants (e.g. OPEN_CLAW).
         */
        void command_claw(const std::string input);

        // label published by the object recognizer
        std::string target_detection_label_;
        // horizontal / vertical size (px) of the camera image, read once from
        // CameraInfo; 0 until the first CameraInfo message arrives
        double image_width_;
        double image_height_;
        // desired position of the box centre as a fraction of the distance from
        // the image centre: 0 = keep box centre at image centre
        double center_offset_x_;
        double center_offset_y_;
        // drop the marker once both axis errors drop below this fraction, i.e.
        // the target is centered under the sub
        double drop_alignment_threshold_;

        // most recent target box, valid only while have_detection_ is true
        steelhead_interfaces::msg::DetectionBox latest_detected_box_;
        bool have_detection_;
        // whether the first marker has already been dropped
        bool marker_dropped_;

        bool finished_;

        rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::Client<steelhead_interfaces::srv::ActuatorsCommand>::SharedPtr actuators_client_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::AutomatedCompetitionDropper)

#endif  //STEELHEAD_TASKS__AUTOMATED_COMPETITION_DROPPER

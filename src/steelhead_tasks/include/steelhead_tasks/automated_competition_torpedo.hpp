#ifndef STEELHEAD_TASKS__AUTOMATED_COMPETITION_TORPEDO
#define STEELHEAD_TASKS__AUTOMATED_COMPETITION_TORPEDO

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"
#include "steelhead_interfaces/srv/actuators_command.hpp"

namespace steelhead_tasks
{

    /** Automated task node for firing at the torpedo task.
     *
     * This node subscribes to the bounding boxes published by the object
     * recognizer, and attempts to center the box with the designated marker. 
     * Then it fires the torpedo. It will then fire the other one in a sort of hail mary.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container. Use this as the reference example for
     * other automated task nodes in this package.
     */
    class AutomatedCompetitionTorpedo : public rclcpp::Node
    {

    public:

        /** Construct the automated competition torpedo task node.
         *
         * Declares parameters, subscribes to the detections topic, sets up the
         * hover adjustment and pipeline feedback publishers, and starts the
         * control loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit AutomatedCompetitionTorpedo(const rclcpp::NodeOptions & options);

    private:

        /** Handle an incoming set of detections.
         *
         * Caches the most recent torpedo bounding box (highest confidence box whose
         * class_id matches the torpedo) so the control loop can act on it.
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
         * Computes a movement adjustment from the most recent cv detection and
         * publishes it to the hover_at_depth adjustment topic. Publishes pipeline
         * feedback once the torpedo is judged to have been passed.
         */
        void control_loop();

        /** Fire the torpedo by calling the actuators command service.
         *
         * Sends an asynchronous request to the actuators_command service with the
         * configured torpedo command. The response is handled in a callback so the
         * control loop is never blocked (a synchronous wait would deadlock this
         * component node's single-threaded executor).
         */
        void fire_torpedo(const std::string input);

        // label published by the object recognizer
        std::string target_detection_label_;
        // horizontal size (px) of the camera image, read once from CameraInfo;
        // 0 until the first CameraInfo message arrives
        double image_width_;
        // desired horizontal position of the box centre as a fraction of the
        // distance from center of image: 0 = keep bounding box center at image centre, 1.0 = keep the center at the right edge of the image, -1.0 = left
        double center_offset_;
        // fire the torpedo once |offset_error| drops below this fraction, i.e.
        // the target hole is aligned with the launcher
        double fire_alignment_threshold_;

        // most recent target box, valid only while have_torpedo_ is true
        steelhead_interfaces::msg::DetectionBox latest_detected_box_;
        bool have_detection_;
        // whether the left torpedo has already been fired
        bool left_torpedo_fired_;

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
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::AutomatedCompetitionTorpedo)

#endif  //STEELHEAD_TASKS__AUTOMATED_COMPETITION_TORPEDO
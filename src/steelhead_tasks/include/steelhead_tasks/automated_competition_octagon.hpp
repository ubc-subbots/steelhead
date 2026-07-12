#ifndef STEELHEAD_TASKS__AUTOMATED_COMPETITION_OCTAGON
#define STEELHEAD_TASKS__AUTOMATED_COMPETITION_OCTAGON

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    /** Automated task node for the competition octagon (surface) task.
     *
     * This node subscribes to the bounding boxes published by the object
     * recognizer on the bottom (downward) camera and centers the detection under
     * the sub in both axes. Once centered, it stops centering and applies a
     * sustained upward force to surface inside the octagon.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container. 
     */
    class AutomatedCompetitionOctagon : public rclcpp::Node
    {

    public:

        /** Construct the automated competition octagon task node.
         *
         * Declares parameters, subscribes to the bottom camera detections topic,
         * sets up the hover adjustment and pipeline feedback publishers, and
         * starts the control loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit AutomatedCompetitionOctagon(const rclcpp::NodeOptions & options);

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
         * Centers the detection under the sub, then surfaces with a sustained
         * upward force once centered. Publishes pipeline feedback once when it
         * begins surfacing.
         */
        void control_loop();

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
        // begin surfacing once both axis errors drop below this fraction, i.e.
        // the target is centered under the sub
        double surface_alignment_threshold_;
        // how long (seconds) to apply the upward force before reporting success
        double surface_duration_;

        // most recent target box, valid only while have_detection_ is true
        steelhead_interfaces::msg::DetectionBox latest_detected_box_;
        bool have_detection_;
        // whether we have centered and started surfacing
        bool surfacing_;
        // time surfacing began, used to time the upward push
        rclcpp::Time surface_start_time_;
        // latched once the surfacing push has completed
        bool finished_;

        rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::AutomatedCompetitionOctagon)

#endif  //STEELHEAD_TASKS__AUTOMATED_COMPETITION_OCTAGON

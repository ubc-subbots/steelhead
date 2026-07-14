#ifndef STEELHEAD_TASKS__AUTOMATED_COMPETITION_SLALOM
#define STEELHEAD_TASKS__AUTOMATED_COMPETITION_SLALOM

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_tasks
{

    /** Automated task node for navigating the competition slalom.
     *
     * The slalom is three sets of vertical PVC pipes, each set arranged WHITE on
     * the left, RED in the middle, and WHITE on the right. Only the red pipe is
     * detected, so this node navigates off the red pipe alone: it holds the red
     * pipe at a fixed offset to one side of the image and drives forward, which
     * carries the AUV through the gap on the other side of it.
     *
     * The offset keeps the same sign for every set, so the AUV passes on the same
     * side of the red pipe each time and earns the bonus for it. The sets are
     * offset laterally from each other, which needs no special handling: the node
     * only ever servos on the pipe it can see, and the hover node holds yaw so the
     * AUV strafes across rather than turning.
     *
     * Several sets are in frame at once, so the node tracks the tallest red box
     * rather than the most confident one. The pipes are all the same height, so the
     * tallest box is the nearest pipe, which is the set being approached.
     *
     * A set only counts once the AUV has actually closed on it: the node commits to
     * a pipe when its box grows past commit_box_height_ of the image, and counts the
     * set when that committed pipe leaves the view. Losing a pipe that was never
     * closed on is treated as a detection dropout and the node keeps hunting. The
     * coast after a committed pipe is lost is a timed phase that ignores detections,
     * so the next set coming into view cannot restart the current set's timer. After
     * num_sets_ sets it reports success to the pipeline manager.
     *
     * Depth is not commanded here: the hover node holds whatever depth the
     * pipeline set beforehand, which should be within the 0.9m span of the pipes
     * to earn the depth bonus.
     *
     * It is a component node registered as a plugin so it can be composed into
     * the steelhead_pipeline container.
     */
    class AutomatedCompetitionSlalom : public rclcpp::Node
    {

    public:

        /** Construct the automated competition slalom task node.
         *
         * Declares parameters, subscribes to the detections topic, sets up the
         * hover adjustment and pipeline feedback publishers, and starts the
         * control loop timer.
         *
         * @param options ros2 node options, supplied by the component container.
         */
        explicit AutomatedCompetitionSlalom(const rclcpp::NodeOptions & options);

    private:

        /** Handle an incoming set of detections.
         *
         * Caches the tallest red pipe box so the control loop can act on it. More
         * than one set is usually in frame, and since the pipes are all the same
         * height the tallest box is the nearest pipe. Picking the most confident
         * box instead would let a clean detection of a set further down the slalom
         * pull the AUV off the one in front of it.
         *
         * @param msg the latest detections from the object recognizer.
         */
        void detections_callback(const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg);

        /** Handle camera calibration info.
         *
         * Reads the image size from the camera's CameraInfo (populated from the
         * camera calibration file) so the control loop knows where the image
         * centre is and how tall a box is relative to the frame. The resolution
         * does not change at runtime, so this reads the size once and then resets
         * the subscription to stop listening.
         *
         * @param msg the camera info for the front camera.
         */
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /** Control loop, called on a fixed timer.
         *
         * Steers to hold the red pipe at the offset, counts sets as they are
         * passed, and publishes pipeline feedback once every set has been cleared.
         */
        void control_loop();

        // label published by the object recognizer for the red slalom pipe
        std::string target_detection_label_;
        // which side of the red pipe to pass on, "left" or "right" as seen in the
        // image. Held constant across every set to earn the same-side bonus.
        std::string pass_side_;
        // how far to hold the red pipe off the image centre, as a fraction of the
        // distance from centre to edge: 0 = keep the pipe centred (and hit it),
        // 1.0 = keep it at the image edge. Larger means a wider berth.
        double pass_offset_;
        // number of pipe sets in the slalom
        int num_sets_;
        // size (px) of the camera image, read once from CameraInfo; 0 until the
        // first CameraInfo message arrives
        double image_width_;
        double image_height_;
        // box height, as a fraction of the image height, at which the AUV is judged
        // close enough to a pipe to commit to it. Only a committed pipe going out
        // of view counts as a passed set, so a detection dropout at range cannot
        // be mistaken for a pass.
        double commit_box_height_;
        // seconds to keep driving after the committed pipe is lost before the set
        // is considered passed
        double pass_timeout_;
        // forward force while searching for and approaching a set
        double approach_force_;
        // forward force while coasting through a set after losing the red pipe
        double coast_force_;
        // sway force per unit of normalized horizontal error
        double sway_gain_;

        // desired position of the red pipe in the image, as a signed fraction of
        // half the image width. Derived from pass_side_ and pass_offset_: passing
        // on the pipe's left means holding the pipe right of centre, so positive.
        double center_offset_;

        // tallest red pipe box from the most recent detections, valid only while
        // have_red_ is true
        steelhead_interfaces::msg::DetectionBox latest_red_pipe_;
        bool have_red_;
        // whether the AUV has closed on the current set's pipe, latched once its
        // box grows past commit_box_height_ and cleared when the set is passed
        bool committed_;
        // whether the committed pipe has gone out of view and the AUV is coasting
        // clear of the set. Detections are ignored while this is true, so the next
        // set coming into view cannot restart the current set's timer.
        bool coasting_;
        rclcpp::Time coast_start_time_;

        // how many sets have been cleared so far
        int sets_passed_;
        bool finished_;

        rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    };

} // namespace steelhead_tasks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_tasks::AutomatedCompetitionSlalom)

#endif  //STEELHEAD_TASKS__AUTOMATED_COMPETITION_SLALOM

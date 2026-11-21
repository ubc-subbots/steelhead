#ifndef STEELHEAD_MISSION__TASKS__PATH_TASK
#define STEELHEAD_MISSION__TASKS__PATH_TASK

#include <algorithm>
#include <cmath>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/msg/path_marker.hpp"


namespace mission
{


    /** Behavior tree action node for following an orange path marker on the pool floor.
     *
     * Uses PathMarker detections from the bottom camera. Drives forward until the
     * marker is under the camera, turns to line up with it while staying centred over
     * it, drives along it, and succeeds once the marker has passed out of view behind
     * the robot. The robot is left holding the marker's heading, which points to the
     * next task. Fails if the marker is lost any other way.
     */
    class PathTask : public BT::StatefulActionNode
    {
        public:


            /** Construct the PathTask action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for subscriptions, publishing, and clock access.
             */
            PathTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by PathTask in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Read port configuration and start searching for the marker.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus onStart() override;


            /** Execute one control step of the current stage.
             *
             * @return BT::NodeStatus::RUNNING while searching or following,
             *         BT::NodeStatus::SUCCESS once the marker has been followed to its end,
             *         or BT::NodeStatus::FAILURE if the marker was lost.
             */
            BT::NodeStatus onRunning() override;


            /** Publish a zero PARTIAL wrench when halted by a timeout or fallback.
             */
            void onHalted() override;


        private:


            /** Stages of following the path, in order. */
            enum class Stage
            {
                SEARCH,
                FOLLOW,
                EXIT
            };


            /** Cache the latest detection in which the marker was visible.
             *
             * @param msg Latest path marker detection from the bottom camera.
             */
            void detectionCallback(const steelhead_interfaces::msg::PathMarker::SharedPtr msg);


            /** Drive forward until the marker is under the camera.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus search();


            /** Turn to line up with the marker, stay centred over it, and drive along it.
             *
             * @return BT::NodeStatus::RUNNING while following, or BT::NodeStatus::FAILURE
             *         if the marker was lost without passing behind the robot.
             */
            BT::NodeStatus follow();


            /** Drive forward for exit_drive_time seconds to clear the end of the marker.
             *
             * @return BT::NodeStatus::RUNNING until the time is up, then BT::NodeStatus::SUCCESS.
             */
            BT::NodeStatus leavePath();


            /** Publish a PARTIAL HoverAdjustment message.
             *
             * @param force_x Forward force.
             * @param force_y Sideways force, positive to the left.
             * @param change_yaw Whether to change the held heading.
             * @param yaw_offset Heading change in radians, positive to the left.
             */
            void publishAdjustment(double force_x, double force_y, bool change_yaw, double yaw_offset);


            /** Publish a zero PARTIAL HoverAdjustment message.
             */
            void publishZeroWrench();


            const double YAW_GAIN = 0.3;              // fraction of the heading error turned per detection
            const double MAX_YAW_STEP = 0.15;         // largest heading change per detection (rad)
            const double FOUND_CENTROID_Y = -0.2;     // marker centre this far down the image counts as under the camera
            const double RELIABLE_CENTROID_Y = 0.5;   // marker direction is only trusted with its centre this close to the middle
            const double EXIT_CENTROID_Y = 0.3;       // marker last seen this far behind centre counts as passed

            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<steelhead_interfaces::msg::PathMarker>::SharedPtr detection_sub_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;

            double speed_;
            double sway_gain_;
            double align_tolerance_;
            double lost_timeout_;
            double exit_drive_time_;

            Stage stage_;
            steelhead_interfaces::msg::PathMarker latest_detection_;
            bool fresh_detection_;
            bool aligned_;
            rclcpp::Time last_seen_time_;
            rclcpp::Time exit_start_time_;
    };


} // namespace mission

#endif  //STEELHEAD_MISSION__TASKS__PATH_TASK

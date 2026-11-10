#ifndef STEELHEAD_MISSION__TASKS__OCTAGON_TASK
#define STEELHEAD_MISSION__TASKS__OCTAGON_TASK

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"


namespace mission
{


    /** Behavior tree action node for the competition octagon (surfacing) task.
     *
     * Subscribes to bottom camera detections and CameraInfo on the shared ROS 2 node,
     * centers the target detection under the AUV in both axes, and applies a timed
     * upward force to surface inside the octagon once centered.
     */
    class OctagonTask : public BT::StatefulActionNode
    {
        public:


            /** Construct the OctagonTask action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for subscriptions, publishing, and clock access.
             */
            OctagonTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by OctagonTask in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Reset per-run octagon state and read port configuration when the node starts.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus onStart() override;


            /** Execute one control step toward centering over the octagon and surfacing.
             *
             * @return BT::NodeStatus::RUNNING while centering or surfacing, or
             *         BT::NodeStatus::SUCCESS once the surfacing push completes.
             */
            BT::NodeStatus onRunning() override;


            /** Publish a zero PARTIAL wrench when halted by a timeout or fallback.
             */
            void onHalted() override;


        private:


            /** Cache the latest bottom camera detection array.
             *
             * @param msg Latest detections from the bottom YOLO detector.
             */
            void detectionsCallback(
                const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg);


            /** Read the image width and height once from the bottom camera CameraInfo topic.
             *
             * @param msg Camera calibration info for the bottom camera.
             */
            void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);


            /** Publish a zero PARTIAL HoverAdjustment message.
             */
            void publishZeroWrench();


            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;

            std::string target_detection_label_;
            double image_width_;
            double image_height_;
            double center_offset_x_;
            double center_offset_y_;
            double surface_alignment_threshold_;
            double surface_duration_;

            steelhead_interfaces::msg::DetectionBoxArray::SharedPtr latest_detections_;
            steelhead_interfaces::msg::DetectionBox latest_detected_box_;
            bool have_detection_;
            bool surfacing_;
            rclcpp::Time surface_start_time_;
    };


} // namespace mission

#endif  //STEELHEAD_MISSION__TASKS__OCTAGON_TASK

#ifndef STEELHEAD_TASKS__BT__GATE_TASK
#define STEELHEAD_TASKS__BT__GATE_TASK

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"


namespace tasks
{


    /** Behavior tree action node for aligning with and passing through the competition gate.
     *
     * Ports AutomatedCompetitionGate into a BT::StatefulActionNode. Subscribes to front
     * camera detections and CameraInfo on the shared ROS 2 node, centers the target
     * detection at the desired horizontal offset while driving forward, and coasts
     * forward for pass_timeout seconds once the gate leaves the field of view.
     */
    class GateTask : public BT::StatefulActionNode
    {
        public:


            /** Construct the GateTask action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for subscriptions, publishing, and clock access.
             */
            GateTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by GateTask in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Reset per-run gate state and read port configuration when the node starts.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus onStart() override;


            /** Execute one control step toward aligning with and passing the gate.
             *
             * @return BT::NodeStatus::RUNNING while approaching/coasting, or
             *         BT::NodeStatus::SUCCESS once the gate has been passed.
             */
            BT::NodeStatus onRunning() override;


            /** Publish a zero PARTIAL wrench when halted by a timeout or fallback.
             */
            void onHalted() override;


        private:


            /** Cache the latest front camera detection array.
             *
             * @param msg Latest detections from the front YOLO detector.
             */
            void detectionsCallback(
                const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg);


            /** Read the image width once from the front camera CameraInfo topic.
             *
             * @param msg Camera calibration info for the front camera.
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
            double pass_timeout_;
            double center_offset_;

            steelhead_interfaces::msg::DetectionBoxArray::SharedPtr latest_detections_;
            steelhead_interfaces::msg::DetectionBox latest_detected_box_;
            bool have_gate_;
            bool have_seen_target_;
            rclcpp::Time last_detection_time_;
    };


} // namespace tasks

#endif  //STEELHEAD_TASKS__BT__GATE_TASK

#ifndef STEELHEAD_MISSION__TASKS__TORPEDO_TASK
#define STEELHEAD_MISSION__TASKS__TORPEDO_TASK

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "steelhead_interfaces/msg/detection_box.hpp"
#include "steelhead_interfaces/msg/detection_box_array.hpp"
#include "steelhead_interfaces/msg/hover_adjustment.hpp"
#include "steelhead_interfaces/srv/actuators_command.hpp"


namespace mission
{


    /** Behavior tree action node for aligning with the torpedo target and firing both torpedoes.
     *
     * Subscribes to front camera detections and CameraInfo on the shared ROS 2 node,
     * sways to center the target detection at center_offset, and fires the left and
     * right torpedoes via the actuators_command service once aligned.
     */
    class TorpedoTask : public BT::StatefulActionNode
    {
        public:


            /** Construct the TorpedoTask action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for subscriptions, publishing, and service client.
             */
            TorpedoTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by TorpedoTask in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Reset per-run torpedo state and read port configuration when the node starts.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus onStart() override;


            /** Execute one control step toward aligning with the target and firing torpedoes.
             *
             * @return BT::NodeStatus::RUNNING while aligning, or BT::NodeStatus::SUCCESS once fired.
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


            /** Send an asynchronous torpedo fire command to the actuators_command service.
             *
             * @param input ActuatorsCommand request string (FIRE_LEFT_TORPEDO or FIRE_RIGHT_TORPEDO).
             */
            void fireTorpedo(const std::string & input);


            /** Publish a zero PARTIAL HoverAdjustment message.
             */
            void publishZeroWrench();


            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<steelhead_interfaces::msg::DetectionBoxArray>::SharedPtr detections_sub_;
            rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
            rclcpp::Client<steelhead_interfaces::srv::ActuatorsCommand>::SharedPtr actuators_client_;

            std::string target_detection_label_;
            double image_width_;
            double center_offset_;
            double fire_alignment_threshold_;

            steelhead_interfaces::msg::DetectionBoxArray::SharedPtr latest_detections_;
            steelhead_interfaces::msg::DetectionBox latest_detected_box_;
            bool have_detection_;
            bool left_torpedo_fired_;
    };


} // namespace mission

#endif  //STEELHEAD_MISSION__TASKS__TORPEDO_TASK

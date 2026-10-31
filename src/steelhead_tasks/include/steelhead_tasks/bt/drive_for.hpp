#ifndef STEELHEAD_TASKS__BT__DRIVE_FOR
#define STEELHEAD_TASKS__BT__DRIVE_FOR

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "steelhead_interfaces/msg/hover_adjustment.hpp"


namespace tasks
{


    /** Behavior tree action node that publishes a constant HoverAdjustment wrench for a duration.
     *
     * Generalizes HardcodeInputs into a BT::StatefulActionNode whose force, torque,
     * and duration parameters are supplied via XML input ports. On completion or
     * preemption (onHalted), a zero PARTIAL wrench is published so hover_at_depth
     * returns to neutral station-keeping.
     */
    class DriveFor : public BT::StatefulActionNode
    {
        public:


            /** Construct the DriveFor action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for publishing and clock access.
             */
            DriveFor(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by DriveFor in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Latch the start time and target wrench from input ports when the node starts.
             *
             * @return BT::NodeStatus::RUNNING if duration > 0, else BT::NodeStatus::SUCCESS.
             */
            BT::NodeStatus onStart() override;


            /** Publish the target wrench while the duration has not elapsed.
             *
             * @return BT::NodeStatus::RUNNING until duration elapses, then BT::NodeStatus::SUCCESS.
             */
            BT::NodeStatus onRunning() override;


            /** Publish a zero PARTIAL wrench when halted by a timeout or fallback.
             */
            void onHalted() override;


        private:


            /** Publish a zero PARTIAL HoverAdjustment message.
             */
            void publishZeroWrench();


            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
            rclcpp::Time start_time_;
            double duration_;
            steelhead_interfaces::msg::HoverAdjustment active_adjustment_;
    };


} // namespace tasks

#endif  //STEELHEAD_TASKS__BT__DRIVE_FOR

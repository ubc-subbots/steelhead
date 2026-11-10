#ifndef STEELHEAD_MISSION__TASKS__PITCH_FLIP_TASK
#define STEELHEAD_MISSION__TASKS__PITCH_FLIP_TASK

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

#include "steelhead_interfaces/msg/hover_adjustment.hpp"


namespace mission
{


    /** Behavior tree action node that performs a single pitch flip using IMU orientation.
     *
     * Latches the starting orientation on the first IMU reading after onStart(),
     * commands a FULL HoverAdjustment pitch torque until the accumulated pitch
     * rotation reaches flip_angle (default 3*pi/2), and then hands back control
     * with a zero PARTIAL wrench so hover_at_depth levels out the remaining quarter turn.
     */
    class PitchFlipTask : public BT::StatefulActionNode
    {
        public:


            /** Construct the PitchFlipTask action node.
             *
             * @param name XML node instance name.
             * @param config BehaviorTree.CPP node configuration and ports.
             * @param node Shared ROS 2 node used for IMU subscription and publishing.
             */
            PitchFlipTask(
                const std::string & name,
                const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);


            /** Declare the input ports accepted by PitchFlipTask in mission XMLs.
             *
             * @return List of input ports with default values.
             */
            static BT::PortsList providedPorts();


            /** Reset accumulated pitch and read port configuration when the node starts.
             *
             * @return BT::NodeStatus::RUNNING.
             */
            BT::NodeStatus onStart() override;


            /** Command the flip torque until the target flip angle is reached.
             *
             * @return BT::NodeStatus::RUNNING while flipping, or BT::NodeStatus::SUCCESS once reached.
             */
            BT::NodeStatus onRunning() override;


            /** Publish a zero PARTIAL wrench when halted by a timeout or fallback.
             */
            void onHalted() override;


        private:


            /** Accumulate pitch rotation across consecutive IMU messages while active.
             *
             * @param msg Latest IMU orientation message.
             */
            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);


            /** Publish a zero PARTIAL HoverAdjustment message.
             */
            void publishZeroWrench();


            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;

            double flip_torque_;
            double flip_angle_;
            bool active_;
            bool started_;
            double accumulated_pitch_;
            tf2::Quaternion prev_orientation_;
    };


} // namespace mission

#endif  //STEELHEAD_MISSION__TASKS__PITCH_FLIP_TASK

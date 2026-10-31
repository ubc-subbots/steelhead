#ifndef STEELHEAD_MISSION__MISSION_RUNNER
#define STEELHEAD_MISSION__MISSION_RUNNER

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "steelhead_interfaces/msg/hover_adjustment.hpp"


namespace mission
{


    /** BehaviorTree.CPP v4 mission runner node for the Steelhead AUV.
     *
     * Loads a behavior tree XML file, registers all task nodes from steelhead_tasks,
     * ticks the tree at a fixed rate, streams live status to Groot2, and publishes
     * both a string status topic and a 3D RViz text marker. Guarantees a zero PARTIAL
     * wrench is published when the mission completes or the node shuts down.
     */
    class MissionRunner : public rclcpp::Node
    {
        public:


            /** Construct the MissionRunner node and declare parameters.
             *
             * @param options ROS 2 node options.
             */
            explicit MissionRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


            /** Register task nodes, load the configured XML tree, and run the tick loop.
             *
             * Must be called after the MissionRunner is owned by a std::shared_ptr so
             * shared_from_this() can be passed to registered BehaviorTree.CPP nodes.
             *
             * @return 0 if the tree finished with SUCCESS, 1 otherwise.
             */
            int run();


        private:


            /** Resolve the configured mission_file parameter to a filesystem path.
             *
             * Absolute or existing paths are used directly; bare filenames are resolved
             * inside the steelhead_mission share/missions directory.
             *
             * @param mission_file Parameter value.
             * @return Resolved filesystem path to the XML file.
             */
            std::string resolveMissionPath(const std::string & mission_file) const;


            /** Collect the names of all currently RUNNING leaf nodes in the tree.
             *
             * @return Formatted string describing the active behavior tree node(s).
             */
            std::string formatActiveStatus(BT::NodeStatus root_status) const;


            /** Publish the current tree status to mission/state and mission/status_marker.
             *
             * @param status_text Human-readable status string.
             * @param root_status Root status of the behavior tree.
             */
            void publishStatus(const std::string & status_text, BT::NodeStatus root_status);


            /** Publish a zero PARTIAL HoverAdjustment wrench to stop thruster overrides.
             */
            void publishZeroWrench();


            std::string mission_file_;
            double tick_rate_hz_;
            int groot_port_;
            std::string marker_frame_;

            BT::BehaviorTreeFactory factory_;
            BT::Tree tree_;
            std::unique_ptr<BT::StdCoutLogger> cout_logger_;
            std::unique_ptr<BT::Groot2Publisher> groot_publisher_;

            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
            rclcpp::Publisher<steelhead_interfaces::msg::HoverAdjustment>::SharedPtr hover_adjust_pub_;
    };


} // namespace mission

#endif  //STEELHEAD_MISSION__MISSION_RUNNER

#include "steelhead_mission/register_nodes.hpp"

#include "steelhead_mission/tasks/drive_for.hpp"
#include "steelhead_mission/tasks/gate_task.hpp"

namespace mission
{


    void registerTaskNodes(
        BT::BehaviorTreeFactory & factory,
        const rclcpp::Node::SharedPtr & node)
    {
        factory.registerNodeType<GateTask>("Gate", node);
        factory.registerNodeType<DriveFor>("DriveFor", node);
    }


} // namespace mission

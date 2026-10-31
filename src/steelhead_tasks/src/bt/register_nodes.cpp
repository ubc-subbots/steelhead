#include "steelhead_tasks/bt/register_nodes.hpp"

#include "steelhead_tasks/bt/drive_for.hpp"
#include "steelhead_tasks/bt/gate_task.hpp"

namespace tasks
{


    void registerTaskNodes(
        BT::BehaviorTreeFactory & factory,
        const rclcpp::Node::SharedPtr & node)
    {
        factory.registerNodeType<GateTask>("Gate", node);
        factory.registerNodeType<DriveFor>("DriveFor", node);
    }


} // namespace tasks

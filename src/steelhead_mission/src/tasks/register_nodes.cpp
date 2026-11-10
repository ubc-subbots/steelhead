#include "steelhead_mission/tasks/register_nodes.hpp"

#include "steelhead_mission/tasks/drive_for.hpp"
#include "steelhead_mission/tasks/dropper_task.hpp"
#include "steelhead_mission/tasks/gate_task.hpp"
#include "steelhead_mission/tasks/octagon_task.hpp"
#include "steelhead_mission/tasks/pitch_flip_task.hpp"
#include "steelhead_mission/tasks/torpedo_task.hpp"

namespace mission
{


    void registerTaskNodes(
        BT::BehaviorTreeFactory & factory,
        const rclcpp::Node::SharedPtr & node)
    {
        factory.registerNodeType<GateTask>("Gate", node);
        factory.registerNodeType<TorpedoTask>("Torpedo", node);
        factory.registerNodeType<DropperTask>("Dropper", node);
        factory.registerNodeType<OctagonTask>("Octagon", node);
        factory.registerNodeType<PitchFlipTask>("PitchFlip", node);
        factory.registerNodeType<DriveFor>("DriveFor", node);
    }


} // namespace mission

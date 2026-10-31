#ifndef STEELHEAD_TASKS__BT__REGISTER_NODES
#define STEELHEAD_TASKS__BT__REGISTER_NODES

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"


namespace tasks
{


    /** Register all steelhead_tasks BehaviorTree.CPP nodes with a factory.
     *
     * Single registration entry point for mission trees. Each task node receives
     * the shared ROS 2 node for subscriptions, publishers, and clock access.
     *
     * @param factory BehaviorTreeFactory to register node builders into.
     * @param node Shared ROS 2 node passed to each constructed BT node.
     */
    void registerTaskNodes(
        BT::BehaviorTreeFactory & factory,
        const rclcpp::Node::SharedPtr & node);


} // namespace tasks

#endif  //STEELHEAD_TASKS__BT__REGISTER_NODES

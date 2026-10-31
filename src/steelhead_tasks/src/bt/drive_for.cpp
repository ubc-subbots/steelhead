#include "steelhead_tasks/bt/drive_for.hpp"

namespace tasks
{


    DriveFor::DriveFor(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      duration_(0.0)
    {
        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );
    }


    BT::PortsList DriveFor::providedPorts()
    {
        return {
            BT::InputPort<double>("force_x", 0.0, "Surge force (N)"),
            BT::InputPort<double>("force_y", 0.0, "Sway force (N)"),
            BT::InputPort<double>("force_z", 0.0, "Heave force (N)"),
            BT::InputPort<double>("torque_x", 0.0, "Roll torque (Nm)"),
            BT::InputPort<double>("torque_y", 0.0, "Pitch torque (Nm)"),
            BT::InputPort<double>("torque_z", 0.0, "Yaw torque (Nm)"),
            BT::InputPort<double>("duration", 1.0, "Duration to apply wrench (seconds)")
        };
    }


    BT::NodeStatus DriveFor::onStart()
    {
        duration_ = 1.0;
        getInput("duration", duration_);

        active_adjustment_ = steelhead_interfaces::msg::HoverAdjustment();
        active_adjustment_.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        getInput("force_x", active_adjustment_.input.force.x);
        getInput("force_y", active_adjustment_.input.force.y);
        getInput("force_z", active_adjustment_.input.force.z);
        getInput("torque_x", active_adjustment_.input.torque.x);
        getInput("torque_y", active_adjustment_.input.torque.y);
        getInput("torque_z", active_adjustment_.input.torque.z);

        start_time_ = node_->now();

        if (duration_ <= 0.0)
        {
            publishZeroWrench();
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "[DriveFor] Starting leg (fx=%.1f, fy=%.1f, fz=%.1f, tz=%.1f) for %.1f s",
            active_adjustment_.input.force.x,
            active_adjustment_.input.force.y,
            active_adjustment_.input.force.z,
            active_adjustment_.input.torque.z,
            duration_
        );

        hover_adjust_pub_->publish(active_adjustment_);
        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus DriveFor::onRunning()
    {
        double elapsed = (node_->now() - start_time_).seconds();
        if (elapsed < duration_)
        {
            hover_adjust_pub_->publish(active_adjustment_);
            return BT::NodeStatus::RUNNING;
        }

        publishZeroWrench();
        RCLCPP_INFO(node_->get_logger(), "[DriveFor] Completed %.1f s leg.", duration_);
        return BT::NodeStatus::SUCCESS;
    }


    void DriveFor::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[DriveFor] Halted; zeroed wrench.");
    }


    void DriveFor::publishZeroWrench()
    {
        auto zero = steelhead_interfaces::msg::HoverAdjustment();
        zero.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(zero);
    }


} // namespace tasks

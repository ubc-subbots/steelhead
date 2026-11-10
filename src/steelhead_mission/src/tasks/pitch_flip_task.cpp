#include "steelhead_mission/tasks/pitch_flip_task.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.hpp>

using std::placeholders::_1;

namespace mission
{


    PitchFlipTask::PitchFlipTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      flip_torque_(1.0),
      flip_angle_(3.0 * M_PI / 2.0),
      active_(false),
      started_(false),
      accumulated_pitch_(0.0)
    {
        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "drivers/imu/out", 10,
            std::bind(&PitchFlipTask::imuCallback, this, _1)
        );
    }


    BT::PortsList PitchFlipTask::providedPorts()
    {
        return {
            BT::InputPort<double>("flip_torque", 1.0, "Pitch torque while flipping (Nm)"),
            BT::InputPort<double>("flip_angle", 4.71238898038469, "Accumulated pitch angle before handoff (rad, default 3*pi/2)")
        };
    }


    BT::NodeStatus PitchFlipTask::onStart()
    {
        flip_torque_ = 1.0;
        flip_angle_ = 3.0 * M_PI / 2.0;

        getInput("flip_torque", flip_torque_);
        getInput("flip_angle", flip_angle_);

        started_ = false;
        accumulated_pitch_ = 0.0;
        active_ = true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[PitchFlipTask] Started (flip_torque=%.2f, flip_angle=%.2f rad)",
            flip_torque_,
            flip_angle_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus PitchFlipTask::onRunning()
    {
        if (!started_)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[PitchFlipTask] Waiting for initial IMU message..."
            );
            return BT::NodeStatus::RUNNING;
        }

        if (std::abs(accumulated_pitch_) < flip_angle_)
        {
            auto adjustment = steelhead_interfaces::msg::HoverAdjustment();
            adjustment.type = steelhead_interfaces::msg::HoverAdjustment::FULL;
            adjustment.input.torque.y = flip_torque_;
            hover_adjust_pub_->publish(adjustment);
            return BT::NodeStatus::RUNNING;
        }

        active_ = false;
        publishZeroWrench();
        RCLCPP_INFO(
            node_->get_logger(),
            "[PitchFlipTask] Pitch flip complete (%.2f rad), hover node taking over.",
            accumulated_pitch_
        );
        return BT::NodeStatus::SUCCESS;
    }


    void PitchFlipTask::onHalted()
    {
        active_ = false;
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[PitchFlipTask] Halted; zeroed wrench.");
    }


    void PitchFlipTask::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!active_)
        {
            return;
        }

        tf2::Quaternion q_current(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        if (!started_)
        {
            prev_orientation_ = q_current;
            started_ = true;
            RCLCPP_INFO(node_->get_logger(), "[PitchFlipTask] Start heading latched, flipping...");
            return;
        }

        tf2::Quaternion q_delta = prev_orientation_.inverse() * q_current;
        q_delta.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q_delta).getRPY(roll, pitch, yaw);
        accumulated_pitch_ += pitch;
        prev_orientation_ = q_current;
    }


    void PitchFlipTask::publishZeroWrench()
    {
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }


} // namespace mission

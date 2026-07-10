#include "steelhead_tasks/pitch_flip.hpp"

#include <cmath>
using std::placeholders::_1;

namespace steelhead_tasks
{

    PitchFlip::PitchFlip(const rclcpp::NodeOptions &options)
        : Node("pitch_flip", options), done_(false), started_(false), accumulated_pitch_(0.0)
    {
        this->declare_parameter<double>("flip_torque", 1.0);
        this->declare_parameter<double>("flip_angle", 3.0 * M_PI / 2.0);

        adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );

        feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
            "/steelhead/pipeline_feedback", 10
        );

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "drivers/imu/out", 10, std::bind(&PitchFlip::imu_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Pitch flip successfully started!");
    }

    void PitchFlip::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (done_) return;

        tf2::Quaternion q_current(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        q_current.normalize();

        if (!started_) {
            start_orientation_ = q_current;
            prev_orientation_ = q_current;
            started_ = true;
            RCLCPP_INFO(this->get_logger(), "Start heading latched, flipping...");
        } else {
            // Deltas between consecutive IMU messages are small, so the
            // body-y component of the delta's rotation vector is the pitch
            // rotated since the last message, free of gimbal lock and
            // accumulating past 2pi
            tf2::Quaternion q_delta = prev_orientation_.inverse() * q_current;
            q_delta.normalize();
            // q and -q encode the same rotation; raw sensor quaternions can
            // hop hemispheres between messages, which would otherwise read
            // as a ~2pi delta
            if (q_delta.w() < 0.0) q_delta = tf2::Quaternion(-q_delta.x(), -q_delta.y(), -q_delta.z(), -q_delta.w());
            accumulated_pitch_ += q_delta.getAngle() * q_delta.getAxis().y();
            prev_orientation_ = q_current;
        }

        double flip_torque, flip_angle;
        this->get_parameter("flip_torque", flip_torque);
        this->get_parameter("flip_angle", flip_angle);

        auto adjustment = steelhead_interfaces::msg::HoverAdjustment();
        if (std::abs(accumulated_pitch_) < flip_angle) {
            adjustment.type = steelhead_interfaces::msg::HoverAdjustment::FULL;
            adjustment.input.torque.y = flip_torque;
            adjust_pub_->publish(adjustment);
        } else {
            // Zero partial adjustment so the hover node levels out the rest
            adjustment.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            adjust_pub_->publish(adjustment);

            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = "Pitch flip almost complete, hover node taking over";
            feedback_pub_->publish(feedback_msg);

            done_ = true;
        }
    }

} // namespace steelhead_tasks

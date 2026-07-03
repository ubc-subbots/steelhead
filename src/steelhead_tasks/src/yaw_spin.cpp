#include "steelhead_tasks/yaw_spin.hpp"

#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

namespace steelhead_tasks
{

namespace
{
    // Wrap an angle into (-pi, pi] so heading differences take the short way
    // around the circle.
    double wrap_pi(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
}


YawSpin::YawSpin(const rclcpp::NodeOptions & options)
: Node("yaw_spin", options), start_yaw_(0.0), yaw_(0.0), prev_yaw_(0.0),
  accumulated_(0.0), have_imu_(false), done_(false)
{
    this->declare_parameter<double>("turns", 1.0);
    // The trim can only command full yaw steps (hover_at_depth uses the sign
    // of torque.z and moves the target a fixed yaw_step), so the tolerance
    // must be wider than one step plus coasting or the vehicle ping-pongs
    // around the start heading forever and the task never completes.
    this->declare_parameter<double>("yaw_tolerance", 0.15);
    this->declare_parameter<std::string>("success_message", "Yaw spin complete");
    this->declare_parameter<double>("control_period", 0.1);

    double control_period = this->get_parameter("control_period").as_double();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "drivers/imu/out", 10,
        std::bind(&YawSpin::imu_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period),
        std::bind(&YawSpin::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Yaw Spin successfully started!");
}


void YawSpin::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (!have_imu_)
    {
        start_yaw_ = yaw;
        prev_yaw_ = yaw;
        have_imu_ = true;
        RCLCPP_INFO(this->get_logger(), "Start heading latched at %.3f rad.", start_yaw_);
    }

    // Accumulate the unwrapped travel so full revolutions are counted rather
    // than lost when the raw yaw wraps at +-pi.
    accumulated_ += wrap_pi(yaw - prev_yaw_);
    prev_yaw_ = yaw;
    yaw_ = yaw;
}


void YawSpin::control_loop()
{
    // Wait until the starting heading is known before commanding anything.
    if (!have_imu_)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for IMU, holding.");
        hover_adjust_pub_->publish(geometry_msgs::msg::Wrench());
        return;
    }

    if (done_)
    {
        // hover_at_depth latches the last adjustment it received, so keep
        // sending a zero wrench until the pipeline manager unloads this node.
        hover_adjust_pub_->publish(geometry_msgs::msg::Wrench());
        return;
    }

    double turns = this->get_parameter("turns").as_double();
    double target_travel = 2.0 * M_PI * std::fabs(turns);
    double direction = turns < 0.0 ? -1.0 : 1.0;

    auto adjust = geometry_msgs::msg::Wrench();

    // Spin phase: keep turning until the full revolutions have been swept.
    // hover_at_depth uses only the sign of torque.z.
    if (std::fabs(accumulated_) < target_travel)
    {
        adjust.torque.z = direction;
        hover_adjust_pub_->publish(adjust);
        return;
    }

    // Trim phase: the spin overshoots the starting heading a little, so steer
    // back onto it before declaring the task done.
    double heading_error = wrap_pi(yaw_ - start_yaw_);
    if (std::fabs(heading_error) > this->get_parameter("yaw_tolerance").as_double())
    {
        adjust.torque.z = heading_error > 0.0 ? -1.0 : 1.0;
        hover_adjust_pub_->publish(adjust);
        return;
    }

    hover_adjust_pub_->publish(geometry_msgs::msg::Wrench());

    auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
    feedback_msg.success = true;
    feedback_msg.message = this->get_parameter("success_message").as_string();
    feedback_pub_->publish(feedback_msg);
    done_ = true;  // don't report success more than once

    RCLCPP_INFO(
        this->get_logger(),
        "Spun %.2f rad, final heading error %.3f rad, reporting success.",
        accumulated_, heading_error
    );
}


} // namespace steelhead_tasks

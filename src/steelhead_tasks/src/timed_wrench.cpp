#include "steelhead_tasks/timed_wrench.hpp"

#include <chrono>

namespace steelhead_tasks
{


TimedWrench::TimedWrench(const rclcpp::NodeOptions & options)
: Node("timed_wrench", options), done_(false)
{
    this->declare_parameter<double>("duration", 10.0);
    this->declare_parameter<double>("force_x", 0.0);
    this->declare_parameter<double>("force_y", 0.0);
    this->declare_parameter<double>("force_z", 0.0);
    this->declare_parameter<double>("torque_x", 0.0);
    this->declare_parameter<double>("torque_y", 0.0);
    this->declare_parameter<double>("torque_z", 0.0);
    this->declare_parameter<std::string>(
        "success_message", "Timed wrench maneuver complete");
    this->declare_parameter<double>("control_period", 0.5);

    double control_period = this->get_parameter("control_period").as_double();

    hover_adjust_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    start_time_ = this->now();
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period),
        std::bind(&TimedWrench::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Timed Wrench successfully started!");
}


void TimedWrench::control_loop()
{
    double duration = this->get_parameter("duration").as_double();
    double elapsed = (this->now() - start_time_).seconds();

    if (elapsed >= duration)
    {
        // hover_at_depth latches the last adjustment it received, so a zero
        // wrench must be sent to end the maneuver. Keep sending it every tick
        // until the pipeline manager unloads this node, in case one is dropped.
        hover_adjust_pub_->publish(geometry_msgs::msg::Wrench());

        if (!done_)
        {
            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = this->get_parameter("success_message").as_string();
            feedback_pub_->publish(feedback_msg);
            done_ = true;  // don't report success more than once
            RCLCPP_INFO(
                this->get_logger(),
                "Wrench held for %.1fs, reporting success.", elapsed
            );
        }
        return;
    }

    auto wrench = geometry_msgs::msg::Wrench();
    wrench.force.x = this->get_parameter("force_x").as_double();
    wrench.force.y = this->get_parameter("force_y").as_double();
    wrench.force.z = this->get_parameter("force_z").as_double();
    wrench.torque.x = this->get_parameter("torque_x").as_double();
    wrench.torque.y = this->get_parameter("torque_y").as_double();
    wrench.torque.z = this->get_parameter("torque_z").as_double();
    hover_adjust_pub_->publish(wrench);
}


} // namespace steelhead_tasks

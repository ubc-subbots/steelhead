#include "steelhead_tasks/hardcode_inputs.hpp"

using namespace std::chrono_literals;

namespace steelhead_tasks
{

    HardcodeInputs::HardcodeInputs(const rclcpp::NodeOptions &options)
        : Node("hardcode_inputs", options), done_(false)
    {
        this->declare_parameter<double>("force_x", 0.0);
        this->declare_parameter<double>("force_y", 0.0);
        this->declare_parameter<double>("force_z", 0.0);
        this->declare_parameter<double>("torque_x", 0.0);
        this->declare_parameter<double>("torque_y", 0.0);
        this->declare_parameter<double>("torque_z", 0.0);
        this->declare_parameter<double>("duration", 5.0);

        adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );

        feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
            "/steelhead/pipeline_feedback", 10
        );

        start_time_ = this->now();
        timer_ = this->create_wall_timer(100ms, std::bind(&HardcodeInputs::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Hardcode inputs successfully started!");
    }

    void HardcodeInputs::timer_callback()
    {
        if (done_) return;

        double duration;
        this->get_parameter("duration", duration);

        auto adjustment = steelhead_interfaces::msg::HoverAdjustment();
        adjustment.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;

        if ((this->now() - start_time_).seconds() < duration) {
            this->get_parameter("force_x", adjustment.input.force.x);
            this->get_parameter("force_y", adjustment.input.force.y);
            this->get_parameter("force_z", adjustment.input.force.z);
            this->get_parameter("torque_x", adjustment.input.torque.x);
            this->get_parameter("torque_y", adjustment.input.torque.y);
            this->get_parameter("torque_z", adjustment.input.torque.z);
            adjust_pub_->publish(adjustment);
        } else {
            // Zero wrench so the hover node goes back to just balancing
            adjust_pub_->publish(adjustment);

            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = "Hardcoded input leg complete";
            feedback_pub_->publish(feedback_msg);

            done_ = true;
            timer_->cancel();
        }
    }

} // namespace steelhead_tasks

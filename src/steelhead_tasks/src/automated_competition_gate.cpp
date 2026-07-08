#include "steelhead_tasks/automated_competition_gate.hpp"

#include <chrono>

using std::placeholders::_1;

namespace steelhead_tasks
{


AutomatedCompetitionGate::AutomatedCompetitionGate(const rclcpp::NodeOptions & options)
: Node("automated_competition_gate", options), image_width_(0.0), have_gate_(false),
  have_seen_target_(false), finished_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "sos");
    this->declare_parameter<double>("pass_timeout", 5.0);
    this->declare_parameter<double>("center_offset", 0.1);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    pass_timeout_ = this->get_parameter("pass_timeout").as_double();
    center_offset_ = this->get_parameter("center_offset").as_double();

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/detections", 10,
        std::bind(&AutomatedCompetitionGate::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "drivers/front_camera/camera_info", 10,
        std::bind(&AutomatedCompetitionGate::camera_info_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&AutomatedCompetitionGate::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Automated Competition Gate succesfully started!");
}


void AutomatedCompetitionGate::detections_callback(
    const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
{
    // Cache the highest-confidence target box, if any, for the control loop to act on
    have_gate_ = false;
    float best_confidence = -1.0f;
    for (const auto & box : msg->boxes)
    {
        if (box.label == target_detection_label_ && box.confidence > best_confidence)
        {
            best_confidence = box.confidence;
            latest_detected_box_ = box;
            have_gate_ = true;
        }
    }
}


void AutomatedCompetitionGate::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    image_width_ = static_cast<double>(msg->width);
    RCLCPP_INFO(this->get_logger(), "Got image width from CameraInfo: %.0f px", image_width_);
    camera_info_sub_.reset();
}


void AutomatedCompetitionGate::control_loop()
{
    if (finished_) return; 

    if (image_width_ <= 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for CameraInfo, holding.");
        auto hold = steelhead_interfaces::msg::HoverAdjustment();
        hold.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(hold);
        return;
    }

    // Target not currently in view.
    if (!have_gate_)
    {
        if (!have_seen_target_)
        {
            auto search = steelhead_interfaces::msg::HoverAdjustment();
            search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            search.input.force.x = 5.0;
            hover_adjust_pub_->publish(search);
            RCLCPP_INFO(this->get_logger(), "Detection box not in sight, going slowly slower to detect it...");
            return;
        }

        // We had the target and just lost it. Keep coasting forward for
        // pass_timeout_ seconds after the loss, then report the gate as passed
        // so the pipeline manager can advance the sequence.
        double since_lost = (this->now() - last_detection_time_).seconds();
        if (since_lost >= pass_timeout_)
        {
            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = "Passed through the competition gate";
            feedback_pub_->publish(feedback_msg);
            finished_ = true;
            RCLCPP_INFO(this->get_logger(), "Passed through gate and a bit further!");

            // publish the stop command for the hover node
            auto stop = steelhead_interfaces::msg::HoverAdjustment();
            stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            hover_adjust_pub_->publish(stop);

            return;
        }

        auto coast = steelhead_interfaces::msg::HoverAdjustment();
        coast.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        coast.input.force.x = 15.0;  // keep driving through the gate
        hover_adjust_pub_->publish(coast);
            RCLCPP_INFO(this->get_logger(), "Continuing to drift past gate...");

        return;
    }

    // Target in view: latch that we have seen it and record the sighting time so
    // the pass timeout above can start counting once it is lost.
    have_seen_target_ = true;
    last_detection_time_ = this->now();

    double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
    double horizontal_error = box_center_x - image_width_ / 2.0;
    double normalized_error = horizontal_error / (image_width_ / 2.0);
    double offset_error = normalized_error - center_offset_;

    auto adjust = steelhead_interfaces::msg::HoverAdjustment();
    adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
    adjust.input.force.x = 5.0;                     // approach the gate
    adjust.input.force.y = -15.0 * offset_error;    // sway toward the offset target

    RCLCPP_INFO(
        this->get_logger(),
        "Gate at center_x=%.1f (offset error=%.2f, target offset=%.2f), swaying and driving forward.",
        box_center_x, offset_error, center_offset_
    );

    hover_adjust_pub_->publish(adjust);
}


} // namespace steelhead_tasks
#include "steelhead_tasks/automated_competition_gate.hpp"

#include <chrono>

using std::placeholders::_1;

namespace steelhead_tasks
{


AutomatedCompetitionGate::AutomatedCompetitionGate(const rclcpp::NodeOptions & options)
: Node("automated_competition_gate", options), image_width_(0.0), have_gate_(false),
  have_seen_target_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "Marker");
    this->declare_parameter<double>("pass_timeout", 2.0);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    pass_timeout_ = this->get_parameter("pass_timeout").as_double();

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
    // Resolution is fixed, so grab the width once and stop listening.
    image_width_ = static_cast<double>(msg->width);
    RCLCPP_INFO(this->get_logger(), "Got image width from CameraInfo: %.0f px", image_width_);
    camera_info_sub_.reset();
}


void AutomatedCompetitionGate::control_loop()
{
    // Wait until we know the image width before trying to centre on the gate.
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
        // Never seen it yet: spin in place to search for the gate. Only yaw is
        // commanded (no forward drive) so the AUV rotates without leaving its
        // position until the gate comes into view. hover_at_depth uses only the
        // sign of torque.z, so search_yaw_ just sets the turn direction.
        if (!have_seen_target_)
        {
            auto search = steelhead_interfaces::msg::HoverAdjustment();
            search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            search.input.force.y = -7.50;
            hover_adjust_pub_->publish(search);
            RCLCPP_INFO(this->get_logger(), "Detection box not in sight, doing slight yaw to find it...");
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
            have_seen_target_ = false;  // don't report the pass more than once
            RCLCPP_INFO(this->get_logger(), "Passed through gate with timeout!");
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

    // Centre the gate horizontally by swaying, and drive forward. The box x is
    // the top-left corner, so the box centre is x + width/2. horizontal_error > 0
    // means the gate is right of image centre; normalise it to [-1, 1] across the
    // image half-width so the command is resolution-independent. hover_at_depth
    // passes force.y straight through as a lateral position error, so this is a
    // proportional sway toward the gate centre. +y is left (REP-103), so a gate
    // right of centre needs a negative y command (flip the sign if the pool shows
    // it swaying the wrong way).
    double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
    double horizontal_error = box_center_x - image_width_ / 2.0;
    double normalized_error = horizontal_error / (image_width_ / 2.0);

    auto adjust = steelhead_interfaces::msg::HoverAdjustment();
    adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
    adjust.input.force.x = 15.0;                       // approach the gate
    adjust.input.force.y = -15.0 * normalized_error;  // sway toward the gate centre

    RCLCPP_INFO(
        this->get_logger(),
        "Gate at center_x=%.1f (normalized error=%.2f), swaying to centre and driving forward.",
        box_center_x, normalized_error
    );

    hover_adjust_pub_->publish(adjust);
}


} // namespace steelhead_tasks
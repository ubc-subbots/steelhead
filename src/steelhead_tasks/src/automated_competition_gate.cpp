#include "steelhead_tasks/automated_competition_gate.hpp"

#include <chrono>

using std::placeholders::_1;

namespace steelhead_tasks
{


AutomatedCompetitionGate::AutomatedCompetitionGate(const rclcpp::NodeOptions & options)
: Node("automated_competition_gate", options), image_width_(0.0), have_gate_(false),
  have_seen_target_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "marker");
    this->declare_parameter<std::string>(
        "camera_info_topic", "drivers/front_camera/camera_info");
    this->declare_parameter<double>("pass_timeout", 2.0);
    this->declare_parameter<double>("forward_drive", 10.0);
    this->declare_parameter<double>("search_yaw", 1.0);
    this->declare_parameter<double>("control_period", 0.5);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    pass_timeout_ = this->get_parameter("pass_timeout").as_double();
    forward_drive_ = this->get_parameter("forward_drive").as_double();
    search_yaw_ = this->get_parameter("search_yaw").as_double();
    double control_period = this->get_parameter("control_period").as_double();

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/detections", 10,
        std::bind(&AutomatedCompetitionGate::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&AutomatedCompetitionGate::camera_info_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period),
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
        hover_adjust_pub_->publish(geometry_msgs::msg::Wrench());
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
            auto search = geometry_msgs::msg::Wrench();
            search.torque.z = search_yaw_;
            hover_adjust_pub_->publish(search);
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
            return;
        }

        auto coast = geometry_msgs::msg::Wrench();
        coast.force.x = forward_drive_;  // keep driving through the gate
        hover_adjust_pub_->publish(coast);
        return;
    }

    // Target in view: latch that we have seen it and record the sighting time so
    // the pass timeout above can start counting once it is lost.
    have_seen_target_ = true;
    last_detection_time_ = this->now();

    // Centre the gate horizontally and drive forward. The box x is the top-left
    // corner, so the box centre is x + width/2. A positive error means the gate
    // is right of centre; hover_at_depth only uses the sign of torque.z for yaw.
    double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
    double horizontal_error = box_center_x - image_width_ / 2.0;

    auto adjust = geometry_msgs::msg::Wrench();
    adjust.force.x = forward_drive_;      // approach the gate
    adjust.torque.z = horizontal_error;   // yaw toward the gate centre

    RCLCPP_INFO(
        this->get_logger(),
        "Gate at center_x=%.1f (error=%.1f), driving forward.",
        box_center_x, horizontal_error
    );

    hover_adjust_pub_->publish(adjust);
}


} // namespace steelhead_tasks

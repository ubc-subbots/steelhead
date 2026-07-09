#include "steelhead_tasks/automated_competition_octagon.hpp"

#include <chrono>
#include <cmath>

using std::placeholders::_1;

namespace steelhead_tasks
{

AutomatedCompetitionOctagon::AutomatedCompetitionOctagon(const rclcpp::NodeOptions & options)
: Node("automated_competition_octagon", options), image_width_(0.0), image_height_(0.0),
  have_detection_(false), surfacing_(false), finished_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "helmet");
    this->declare_parameter<double>("center_offset_x", 0.0);
    this->declare_parameter<double>("center_offset_y", 0.0);
    this->declare_parameter<double>("surface_alignment_threshold", 0.3);
    this->declare_parameter<double>("surface_duration", 10.0);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    center_offset_x_ = this->get_parameter("center_offset_x").as_double();
    center_offset_y_ = this->get_parameter("center_offset_y").as_double();
    surface_alignment_threshold_ = this->get_parameter("surface_alignment_threshold").as_double();
    surface_duration_ = this->get_parameter("surface_duration").as_double();

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/bottom/detections", 10,
        std::bind(&AutomatedCompetitionOctagon::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "drivers/bottom_camera/camera_info", 10,
        std::bind(&AutomatedCompetitionOctagon::camera_info_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&AutomatedCompetitionOctagon::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Automated Competition Octagon succesfully started!");
}


void AutomatedCompetitionOctagon::detections_callback(
    const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
{
    // Cache the highest-confidence target box, if any, for the control loop to act on
    have_detection_ = false;
    float best_confidence = -1.0f;
    for (const auto & box : msg->boxes)
    {
        if (box.label == target_detection_label_ && box.confidence > best_confidence)
        {
            best_confidence = box.confidence;
            latest_detected_box_ = box;
            have_detection_ = true;
        }
    }
}


void AutomatedCompetitionOctagon::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    image_width_ = static_cast<double>(msg->width);
    image_height_ = static_cast<double>(msg->height);
    RCLCPP_INFO(
        this->get_logger(), "Got image size from CameraInfo: %.0f x %.0f px",
        image_width_, image_height_
    );
    camera_info_sub_.reset();
}


void AutomatedCompetitionOctagon::control_loop()
{
    if (finished_) return;

    // Once centered we apply a sustained upward force for surface_duration_
    // seconds, then report success and stop.
    if (surfacing_)
    {
        double elapsed = (this->now() - surface_start_time_).seconds();
        if (elapsed >= surface_duration_)
        {
            auto stop = steelhead_interfaces::msg::HoverAdjustment();
            stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            hover_adjust_pub_->publish(stop);

            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = "Surfaced inside the octagon!";
            feedback_pub_->publish(feedback_msg);
            finished_ = true;
            RCLCPP_INFO(this->get_logger(), "Finished surfacing inside the octagon!");
            return;
        }

        auto surface = steelhead_interfaces::msg::HoverAdjustment();
        surface.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        surface.input.force.z = 15.0;   
        hover_adjust_pub_->publish(surface);
        RCLCPP_INFO(this->get_logger(), "Surfacing inside the octagon... (%.1fs / %.1fs)", elapsed, surface_duration_);
        return;
    }

    if (image_width_ <= 0.0 || image_height_ <= 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for CameraInfo, holding.");
        auto hold = steelhead_interfaces::msg::HoverAdjustment();
        hold.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(hold);
        return;
    }

    // Target not currently in view.
    if (!have_detection_)
    {
        auto search = steelhead_interfaces::msg::HoverAdjustment();
        search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        search.input.force.x = 3.0;
        hover_adjust_pub_->publish(search);
        RCLCPP_INFO(this->get_logger(), "Detection box not in sight, drifting to find it...");
        return;
    }

    // Bottom camera looks down: image-x error maps to sway (force.y), image-y
    // error maps to forward/back (force.x).
    double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
    double box_center_y = latest_detected_box_.y + latest_detected_box_.height / 2.0;

    double normalized_error_x = (box_center_x - image_width_ / 2.0) / (image_width_ / 2.0);
    double normalized_error_y = (box_center_y - image_height_ / 2.0) / (image_height_ / 2.0);

    double offset_error_x = normalized_error_x - center_offset_x_;
    double offset_error_y = normalized_error_y - center_offset_y_;

    auto adjust = steelhead_interfaces::msg::HoverAdjustment();
    adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
    adjust.input.force.y = -3.0 * offset_error_x;   // sway toward the offset target
    adjust.input.force.x = 3.0 * offset_error_y;    // move fore/aft toward the target

    RCLCPP_INFO(
        this->get_logger(),
        "Detection at center=(%.1f, %.1f) (offset error x=%.2f, y=%.2f), centering over target...",
        box_center_x, box_center_y, offset_error_x, offset_error_y
    );

    hover_adjust_pub_->publish(adjust);

    // Once centered under the target, start the timed surfacing push. Success is
    // reported later, once the upward force has been applied for surface_duration_.
    if (std::abs(offset_error_x) < surface_alignment_threshold_ &&
        std::abs(offset_error_y) < surface_alignment_threshold_)
    {
        surfacing_ = true;
        surface_start_time_ = this->now();
        RCLCPP_INFO(
            this->get_logger(),
            "Centered over octagon, applying upward force for %.1fs.", surface_duration_
        );
    }
}


} // namespace steelhead_tasks

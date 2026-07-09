#include "steelhead_tasks/automated_competition_dropper.hpp"

#include <chrono>
#include <cmath>

using std::placeholders::_1;

namespace steelhead_tasks
{

AutomatedCompetitionDropper::AutomatedCompetitionDropper(const rclcpp::NodeOptions & options)
: Node("automated_competition_dropper", options), image_width_(0.0), image_height_(0.0),
  have_detection_(false), marker_dropped_(false), finished_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "fire");
    this->declare_parameter<double>("center_offset_x", 0.0);
    this->declare_parameter<double>("center_offset_y", 0.1);
    this->declare_parameter<double>("drop_alignment_threshold", 0.1);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    center_offset_x_ = this->get_parameter("center_offset_x").as_double();
    center_offset_y_ = this->get_parameter("center_offset_y").as_double();
    drop_alignment_threshold_ = this->get_parameter("drop_alignment_threshold").as_double();

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/bottom/detections", 10,
        std::bind(&AutomatedCompetitionDropper::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "drivers/bottom_camera/camera_info", 10,
        std::bind(&AutomatedCompetitionDropper::camera_info_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    actuators_client_ = this->create_client<steelhead_interfaces::srv::ActuatorsCommand>(
        "/steelhead/controls/actuators_command"
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&AutomatedCompetitionDropper::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Automated Competition Dropper succesfully started!");
}


void AutomatedCompetitionDropper::detections_callback(
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


void AutomatedCompetitionDropper::camera_info_callback(
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


void AutomatedCompetitionDropper::control_loop()
{
    if (finished_) return;

    command_claw(steelhead_interfaces::srv::ActuatorsCommand::Request::CLOSE_CLAW);

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
        search.input.force.x = 5.0;
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
    adjust.input.force.x = 3.0 * offset_error_y;   // move fore/aft toward the target

    RCLCPP_INFO(
        this->get_logger(),
        "Bin at center=(%.1f, %.1f) (offset error x=%.2f, y=%.2f), centering over target...",
        box_center_x, box_center_y, offset_error_x, offset_error_y
    );

    hover_adjust_pub_->publish(adjust);

    if (!marker_dropped_ &&
        std::abs(offset_error_x) < drop_alignment_threshold_ &&
        std::abs(offset_error_y) < drop_alignment_threshold_)
    {
        command_claw(steelhead_interfaces::srv::ActuatorsCommand::Request::OPEN_CLAW);
        marker_dropped_ = true;
        
        target_detection_label_ = "blood";
        have_detection_ = false;
        return;
    }

    if (marker_dropped_ &&
        std::abs(offset_error_x) < drop_alignment_threshold_ &&
        std::abs(offset_error_y) < drop_alignment_threshold_)
    {
        command_claw(steelhead_interfaces::srv::ActuatorsCommand::Request::OPEN_CLAW);

        finished_ = true;

        auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
        feedback_msg.success = true;
        feedback_msg.message = "Dropped marker on target!";
        feedback_pub_->publish(feedback_msg);
        RCLCPP_INFO(this->get_logger(), "Centered over target and dropped marker!");

        // publish the stop command for the hover node
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }
}


void AutomatedCompetitionDropper::command_claw(const std::string input)
{
    if (!actuators_client_->service_is_ready())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "actuators_command service is not available. Is it running?"
        );
        return;
    }

    auto request = std::make_shared<steelhead_interfaces::srv::ActuatorsCommand::Request>();
    request->input = input;

    actuators_client_->async_send_request(
        request,
        [this](rclcpp::Client<steelhead_interfaces::srv::ActuatorsCommand>::SharedFuture future)
        {
            if (!future.get()->succeeded)
            {
                RCLCPP_ERROR(this->get_logger(), "Actuators service reported the marker drop failed. ");
            }
        }
    );
}


} // namespace steelhead_tasks

#include "steelhead_tasks/automated_competition_torpedo.hpp"

#include <chrono>
#include <cmath>
#include <steelhead_interfaces/srv/detail/actuators_command__struct.hpp>

using std::placeholders::_1;

namespace steelhead_tasks
{

AutomatedCompetitionTorpedo::AutomatedCompetitionTorpedo(const rclcpp::NodeOptions & options)
: Node("automated_competition_torpedo", options), image_width_(0.0), have_detection_(false),
  left_torpedo_fired_(false), finished_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "circle");
    this->declare_parameter<double>("pass_timeout", 4.0);
    this->declare_parameter<double>("center_offset", 0.0);
    this->declare_parameter<double>("fire_alignment_threshold", 0.1);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    center_offset_ = this->get_parameter("center_offset").as_double();
    fire_alignment_threshold_ = this->get_parameter("fire_alignment_threshold").as_double();

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/detections", 10,
        std::bind(&AutomatedCompetitionTorpedo::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "drivers/front_camera/camera_info", 10,
        std::bind(&AutomatedCompetitionTorpedo::camera_info_callback, this, _1)
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
        std::bind(&AutomatedCompetitionTorpedo::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Automated Competition Torpedo succesfully started!");
}


void AutomatedCompetitionTorpedo::detections_callback(
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


void AutomatedCompetitionTorpedo::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    image_width_ = static_cast<double>(msg->width);
    RCLCPP_INFO(this->get_logger(), "Got image width from CameraInfo: %.0f px", image_width_);
    camera_info_sub_.reset();
}


void AutomatedCompetitionTorpedo::control_loop()
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
    if (!have_detection_)
    {
        auto search = steelhead_interfaces::msg::HoverAdjustment();
        search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        search.input.force.x = 5.0;
        hover_adjust_pub_->publish(search);
        RCLCPP_INFO(this->get_logger(), "Detection box not in sight, going slowly slower to detect it...");
        return;
    }

    double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
    double horizontal_error = box_center_x - image_width_ / 2.0;
    double normalized_error = horizontal_error / (image_width_ / 2.0);
    double offset_error = normalized_error - center_offset_;

    auto adjust = steelhead_interfaces::msg::HoverAdjustment();
    adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
    adjust.input.force.y = -15.0 * offset_error;    // sway toward the offset target

    RCLCPP_INFO(
        this->get_logger(),
        "Torpedo at center_x=%.1f (offset error=%.2f, target offset=%.2f), swaying to center bounding box...",
        box_center_x, offset_error, center_offset_
    );

    hover_adjust_pub_->publish(adjust);

    // Once the hole is aligned with the launcher, fire the torpedo (only once).
    if (!left_torpedo_fired_ && std::abs(offset_error) < fire_alignment_threshold_)
    {
        fire_torpedo(steelhead_interfaces::srv::ActuatorsCommand::Request::FIRE_LEFT_TORPEDO);
        left_torpedo_fired_ = true;
    }

    if (left_torpedo_fired_ && std::abs(offset_error) < fire_alignment_threshold_)
    {
        fire_torpedo(steelhead_interfaces::srv::ActuatorsCommand::Request::FIRE_RIGHT_TORPEDO);
        finished_ = true;

        auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
        feedback_msg.success = true;
        feedback_msg.message = "Fired torpedo at target!";
        feedback_pub_->publish(feedback_msg);
        finished_ = true;
        RCLCPP_INFO(this->get_logger(), "Fired both torpedoes!");

        // publish the stop command for the hover node
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }
}


void AutomatedCompetitionTorpedo::fire_torpedo(const std::string input)
{
    if (!actuators_client_->service_is_ready())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Aligned with torpedo target but actuators_command service is not available. Is it running?"
        );
        return;
    }

    auto request = std::make_shared<steelhead_interfaces::srv::ActuatorsCommand::Request>();
    request->input = input;

    RCLCPP_INFO(this->get_logger(), "Aligned with target, firing torpedo: '%s'", input.c_str());

    actuators_client_->async_send_request(
        request,
        [this](rclcpp::Client<steelhead_interfaces::srv::ActuatorsCommand>::SharedFuture future)
        {
            if (future.get()->succeeded)
            {
                RCLCPP_INFO(this->get_logger(), "Torpedo fired successfully.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Actuators service reported the torpedo firing failed. ");
            }
        }
    );
}


} // namespace steelhead_tasks
#include "steelhead_tasks/automated_competition_slalom.hpp"

#include <chrono>

using std::placeholders::_1;

namespace steelhead_tasks
{


AutomatedCompetitionSlalom::AutomatedCompetitionSlalom(const rclcpp::NodeOptions & options)
: Node("automated_competition_slalom", options), image_width_(0.0), image_height_(0.0),
  center_offset_(0.0), have_red_(false), committed_(false), coasting_(false),
  sets_passed_(0), finished_(false)
{
    this->declare_parameter<std::string>("target_detection_label", "slalom");
    this->declare_parameter<std::string>("pass_side", "left");
    this->declare_parameter<double>("pass_offset", 0.4);
    this->declare_parameter<int>("num_sets", 3);
    this->declare_parameter<double>("commit_box_height", 0.4);
    this->declare_parameter<double>("pass_timeout", 6.0);
    this->declare_parameter<double>("approach_force", 5.0);
    this->declare_parameter<double>("coast_force", 15.0);
    this->declare_parameter<double>("sway_gain", 15.0);

    target_detection_label_ = this->get_parameter("target_detection_label").as_string();
    pass_side_ = this->get_parameter("pass_side").as_string();
    pass_offset_ = this->get_parameter("pass_offset").as_double();
    num_sets_ = this->get_parameter("num_sets").as_int();
    commit_box_height_ = this->get_parameter("commit_box_height").as_double();
    pass_timeout_ = this->get_parameter("pass_timeout").as_double();
    approach_force_ = this->get_parameter("approach_force").as_double();
    coast_force_ = this->get_parameter("coast_force").as_double();
    sway_gain_ = this->get_parameter("sway_gain").as_double();

    if (pass_side_ != "left" && pass_side_ != "right")
    {
        RCLCPP_WARN(
            this->get_logger(),
            "pass_side '%s' is not 'left' or 'right', defaulting to 'left'.", pass_side_.c_str()
        );
        pass_side_ = "left";
    }

    // Passing on the pipe's left means keeping the pipe to the right of centre.
    center_offset_ = (pass_side_ == "left") ? pass_offset_ : -pass_offset_;

    detections_sub_ = this->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
        "yolo_detector/front/detections", 10,
        std::bind(&AutomatedCompetitionSlalom::detections_callback, this, _1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "drivers/front_camera/camera_info", 10,
        std::bind(&AutomatedCompetitionSlalom::camera_info_callback, this, _1)
    );

    hover_adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
        "controls/hover_adjust", 10
    );

    feedback_pub_ = this->create_publisher<steelhead_interfaces::msg::PipelineFeedback>(
        "/steelhead/pipeline_feedback", 10
    );

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.5),
        std::bind(&AutomatedCompetitionSlalom::control_loop, this)
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Automated Competition Slalom succesfully started! Passing %s of the red pipe on %d sets.",
        pass_side_.c_str(), num_sets_
    );
}


void AutomatedCompetitionSlalom::detections_callback(
    const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
{
    // Cache the tallest red pipe box, if any, for the control loop to act on. The
    // pipes are all the same height, so the tallest box is the nearest pipe, which
    // is the set we are approaching.
    have_red_ = false;
    float tallest = -1.0f;
    for (const auto & box : msg->boxes)
    {
        if (box.label == target_detection_label_ && box.height > tallest)
        {
            tallest = box.height;
            latest_red_pipe_ = box;
            have_red_ = true;
        }
    }
}


void AutomatedCompetitionSlalom::camera_info_callback(
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


void AutomatedCompetitionSlalom::control_loop()
{
    if (finished_) return;

    if (image_width_ <= 0.0 || image_height_ <= 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for CameraInfo, holding.");
        auto hold = steelhead_interfaces::msg::HoverAdjustment();
        hold.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(hold);
        return;
    }

    // Coasting clear of the set we just passed. Detections are deliberately not
    // read here: the next set is usually already in view, and acting on it now
    // would restart this set's timer and it would never be counted.
    if (coasting_)
    {
        double elapsed = (this->now() - coast_start_time_).seconds();
        if (elapsed < pass_timeout_)
        {
            auto coast = steelhead_interfaces::msg::HoverAdjustment();
            coast.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            coast.input.force.x = coast_force_;   // keep driving past the pipe
            hover_adjust_pub_->publish(coast);
            RCLCPP_INFO(
                this->get_logger(), "Drifting clear of set %d... (%.1fs / %.1fs)",
                sets_passed_ + 1, elapsed, pass_timeout_
            );
            return;
        }

        sets_passed_++;
        coasting_ = false;
        committed_ = false;
        RCLCPP_INFO(this->get_logger(), "Passed slalom set %d of %d!", sets_passed_, num_sets_);

        if (sets_passed_ >= num_sets_)
        {
            auto stop = steelhead_interfaces::msg::HoverAdjustment();
            stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            hover_adjust_pub_->publish(stop);

            auto feedback_msg = steelhead_interfaces::msg::PipelineFeedback();
            feedback_msg.success = true;
            feedback_msg.message = "Navigated the competition slalom";
            feedback_pub_->publish(feedback_msg);
            finished_ = true;
            RCLCPP_INFO(this->get_logger(), "Cleared every slalom set!");
        }

        return;
    }

    // Red pipe of the current set not in view.
    if (!have_red_)
    {
        // If we had closed on a pipe and it has now left the frame, we are
        // alongside it, so coast clear of the set and count it.
        if (committed_)
        {
            coasting_ = true;
            coast_start_time_ = this->now();
            RCLCPP_INFO(
                this->get_logger(),
                "Committed pipe of set %d left the view, coasting clear for %.1fs.",
                sets_passed_ + 1, pass_timeout_
            );
            return;
        }

        // Never got close to it, so this is a detection dropout or the set is not
        // in view yet. Keep hunting.
        auto search = steelhead_interfaces::msg::HoverAdjustment();
        search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        search.input.force.x = approach_force_;
        hover_adjust_pub_->publish(search);
        RCLCPP_INFO(
            this->get_logger(),
            "Red pipe of set %d not in sight, going slowly forward to detect it...",
            sets_passed_ + 1
        );
        return;
    }

    // Red pipe in view. Once its box fills enough of the frame we are close enough
    // that this is definitely the set in front of us, so commit to it: from here on
    // losing it means we have drawn alongside and passed it.
    double box_height_fraction = latest_red_pipe_.height / image_height_;
    if (!committed_ && box_height_fraction >= commit_box_height_)
    {
        committed_ = true;
        RCLCPP_INFO(
            this->get_logger(), "Closed on set %d (box fills %.2f of the frame), committing to it.",
            sets_passed_ + 1, box_height_fraction
        );
    }

    double pipe_center_x = latest_red_pipe_.x + latest_red_pipe_.width / 2.0;
    double horizontal_error = pipe_center_x - image_width_ / 2.0;
    double normalized_error = horizontal_error / (image_width_ / 2.0);
    double offset_error = normalized_error - center_offset_;

    auto adjust = steelhead_interfaces::msg::HoverAdjustment();
    adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
    adjust.input.force.x = approach_force_;                 // approach the set
    adjust.input.force.y = -sway_gain_ * offset_error;      // sway toward the offset target

    RCLCPP_INFO(
        this->get_logger(),
        "Set %d: red pipe at center_x=%.1f (offset error=%.2f, target offset=%.2f), swaying and driving forward.",
        sets_passed_ + 1, pipe_center_x, offset_error, center_offset_
    );

    hover_adjust_pub_->publish(adjust);
}


} // namespace steelhead_tasks

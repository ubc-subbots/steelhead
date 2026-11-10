#include "steelhead_mission/tasks/octagon_task.hpp"

#include <cmath>

using std::placeholders::_1;

namespace mission
{


    OctagonTask::OctagonTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      target_detection_label_("helmet"),
      image_width_(0.0),
      image_height_(0.0),
      center_offset_x_(0.0),
      center_offset_y_(0.0),
      surface_alignment_threshold_(0.3),
      surface_duration_(10.0),
      have_detection_(false),
      surfacing_(false)
    {
        detections_sub_ = node_->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
            "yolo_detector/bottom/detections", 10,
            std::bind(&OctagonTask::detectionsCallback, this, _1)
        );

        camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "drivers/bottom_camera/camera_info", 10,
            std::bind(&OctagonTask::cameraInfoCallback, this, _1)
        );

        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );
    }


    BT::PortsList OctagonTask::providedPorts()
    {
        return {
            BT::InputPort<std::string>("label", "helmet", "Target detection class label"),
            BT::InputPort<double>("center_offset_x", 0.0, "Desired horizontal box position (-1.0 to 1.0)"),
            BT::InputPort<double>("center_offset_y", 0.0, "Desired vertical box position (-1.0 to 1.0)"),
            BT::InputPort<double>("surface_alignment_threshold", 0.3, "Alignment error threshold to begin surfacing"),
            BT::InputPort<double>("surface_duration", 10.0, "Seconds to apply upward surfacing force")
        };
    }


    BT::NodeStatus OctagonTask::onStart()
    {
        target_detection_label_ = "helmet";
        center_offset_x_ = 0.0;
        center_offset_y_ = 0.0;
        surface_alignment_threshold_ = 0.3;
        surface_duration_ = 10.0;

        getInput("label", target_detection_label_);
        getInput("center_offset_x", center_offset_x_);
        getInput("center_offset_y", center_offset_y_);
        getInput("surface_alignment_threshold", surface_alignment_threshold_);
        getInput("surface_duration", surface_duration_);

        have_detection_ = false;
        surfacing_ = false;
        latest_detections_.reset();

        RCLCPP_INFO(
            node_->get_logger(),
            "[OctagonTask] Started (label='%s', threshold=%.2f, surface_duration=%.1f s)",
            target_detection_label_.c_str(),
            surface_alignment_threshold_,
            surface_duration_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus OctagonTask::onRunning()
    {
        if (surfacing_)
        {
            double elapsed = (node_->now() - surface_start_time_).seconds();
            if (elapsed >= surface_duration_)
            {
                publishZeroWrench();
                RCLCPP_INFO(node_->get_logger(), "[OctagonTask] Finished surfacing inside the octagon!");
                return BT::NodeStatus::SUCCESS;
            }

            auto surface = steelhead_interfaces::msg::HoverAdjustment();
            surface.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            surface.input.force.z = 15.0;
            hover_adjust_pub_->publish(surface);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[OctagonTask] Surfacing inside the octagon... (%.1fs / %.1fs)",
                elapsed, surface_duration_
            );
            return BT::NodeStatus::RUNNING;
        }

        if (image_width_ <= 0.0 || image_height_ <= 0.0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[OctagonTask] Waiting for CameraInfo, holding."
            );
            publishZeroWrench();
            return BT::NodeStatus::RUNNING;
        }

        have_detection_ = false;
        if (latest_detections_)
        {
            float best_confidence = -1.0f;
            for (const auto & box : latest_detections_->boxes)
            {
                if (box.label == target_detection_label_ && box.confidence > best_confidence)
                {
                    best_confidence = box.confidence;
                    latest_detected_box_ = box;
                    have_detection_ = true;
                }
            }
        }

        if (!have_detection_)
        {
            auto search = steelhead_interfaces::msg::HoverAdjustment();
            search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            search.input.force.x = 3.0;
            hover_adjust_pub_->publish(search);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[OctagonTask] Detection box '%s' not in sight, drifting to find it...",
                target_detection_label_.c_str()
            );
            return BT::NodeStatus::RUNNING;
        }

        double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
        double box_center_y = latest_detected_box_.y + latest_detected_box_.height / 2.0;

        double normalized_error_x = (box_center_x - image_width_ / 2.0) / (image_width_ / 2.0);
        double normalized_error_y = (box_center_y - image_height_ / 2.0) / (image_height_ / 2.0);

        double offset_error_x = normalized_error_x - center_offset_x_;
        double offset_error_y = normalized_error_y - center_offset_y_;

        auto adjust = steelhead_interfaces::msg::HoverAdjustment();
        adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        adjust.input.force.y = -3.0 * offset_error_x;
        adjust.input.force.x = 3.0 * offset_error_y;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[OctagonTask] Detection at center=(%.1f, %.1f) (offset error x=%.2f, y=%.2f), centering...",
            box_center_x, box_center_y, offset_error_x, offset_error_y
        );

        hover_adjust_pub_->publish(adjust);

        if (std::abs(offset_error_x) < surface_alignment_threshold_ &&
            std::abs(offset_error_y) < surface_alignment_threshold_)
        {
            surfacing_ = true;
            surface_start_time_ = node_->now();
            RCLCPP_INFO(
                node_->get_logger(),
                "[OctagonTask] Centered over octagon, applying upward force for %.1fs.",
                surface_duration_
            );
        }

        return BT::NodeStatus::RUNNING;
    }


    void OctagonTask::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[OctagonTask] Halted; zeroed wrench.");
    }


    void OctagonTask::detectionsCallback(
        const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
    {
        latest_detections_ = msg;
    }


    void OctagonTask::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        image_width_ = static_cast<double>(msg->width);
        image_height_ = static_cast<double>(msg->height);
        RCLCPP_INFO(
            node_->get_logger(),
            "[OctagonTask] Got image size from CameraInfo: %.0f x %.0f px",
            image_width_, image_height_
        );
        camera_info_sub_.reset();
    }


    void OctagonTask::publishZeroWrench()
    {
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }


} // namespace mission

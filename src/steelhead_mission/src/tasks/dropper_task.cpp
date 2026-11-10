#include "steelhead_mission/tasks/dropper_task.hpp"

#include <cmath>

using std::placeholders::_1;

namespace mission
{


    DropperTask::DropperTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      target_detection_label_("fire"),
      second_detection_label_("blood"),
      image_width_(0.0),
      image_height_(0.0),
      center_offset_x_(0.0),
      center_offset_y_(0.1),
      drop_alignment_threshold_(0.1),
      have_detection_(false),
      marker_dropped_(false)
    {
        detections_sub_ = node_->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
            "yolo_detector/bottom/detections", 10,
            std::bind(&DropperTask::detectionsCallback, this, _1)
        );

        camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "drivers/bottom_camera/camera_info", 10,
            std::bind(&DropperTask::cameraInfoCallback, this, _1)
        );

        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );

        actuators_client_ = node_->create_client<steelhead_interfaces::srv::ActuatorsCommand>(
            "/steelhead/controls/actuators_command"
        );
    }


    BT::PortsList DropperTask::providedPorts()
    {
        return {
            BT::InputPort<std::string>("label", "fire", "Initial target detection class label"),
            BT::InputPort<std::string>("second_label", "blood", "Second target detection class label"),
            BT::InputPort<double>("center_offset_x", 0.0, "Desired horizontal box position (-1.0 to 1.0)"),
            BT::InputPort<double>("center_offset_y", 0.1, "Desired vertical box position (-1.0 to 1.0)"),
            BT::InputPort<double>("drop_alignment_threshold", 0.1, "Alignment error threshold to drop marker")
        };
    }


    BT::NodeStatus DropperTask::onStart()
    {
        target_detection_label_ = "fire";
        second_detection_label_ = "blood";
        center_offset_x_ = 0.0;
        center_offset_y_ = 0.1;
        drop_alignment_threshold_ = 0.1;

        getInput("label", target_detection_label_);
        getInput("second_label", second_detection_label_);
        getInput("center_offset_x", center_offset_x_);
        getInput("center_offset_y", center_offset_y_);
        getInput("drop_alignment_threshold", drop_alignment_threshold_);

        have_detection_ = false;
        marker_dropped_ = false;
        latest_detections_.reset();

        commandClaw(steelhead_interfaces::srv::ActuatorsCommand::Request::CLOSE_CLAW);

        RCLCPP_INFO(
            node_->get_logger(),
            "[DropperTask] Started (label='%s', second_label='%s', threshold=%.2f)",
            target_detection_label_.c_str(),
            second_detection_label_.c_str(),
            drop_alignment_threshold_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus DropperTask::onRunning()
    {
        if (image_width_ <= 0.0 || image_height_ <= 0.0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[DropperTask] Waiting for CameraInfo, holding."
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
            search.input.force.x = 5.0;
            hover_adjust_pub_->publish(search);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[DropperTask] Detection box '%s' not in sight, drifting to find it...",
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
            "[DropperTask] Bin at center=(%.1f, %.1f) (offset error x=%.2f, y=%.2f), centering...",
            box_center_x, box_center_y, offset_error_x, offset_error_y
        );

        hover_adjust_pub_->publish(adjust);

        if (!marker_dropped_ &&
            std::abs(offset_error_x) < drop_alignment_threshold_ &&
            std::abs(offset_error_y) < drop_alignment_threshold_)
        {
            commandClaw(steelhead_interfaces::srv::ActuatorsCommand::Request::OPEN_CLAW);
            marker_dropped_ = true;
            target_detection_label_ = second_detection_label_;
            have_detection_ = false;
            return BT::NodeStatus::RUNNING;
        }

        if (marker_dropped_ &&
            std::abs(offset_error_x) < drop_alignment_threshold_ &&
            std::abs(offset_error_y) < drop_alignment_threshold_)
        {
            commandClaw(steelhead_interfaces::srv::ActuatorsCommand::Request::OPEN_CLAW);
            publishZeroWrench();
            RCLCPP_INFO(node_->get_logger(), "[DropperTask] Centered over target and dropped marker!");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }


    void DropperTask::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[DropperTask] Halted; zeroed wrench.");
    }


    void DropperTask::detectionsCallback(
        const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
    {
        latest_detections_ = msg;
    }


    void DropperTask::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        image_width_ = static_cast<double>(msg->width);
        image_height_ = static_cast<double>(msg->height);
        RCLCPP_INFO(
            node_->get_logger(),
            "[DropperTask] Got image size from CameraInfo: %.0f x %.0f px",
            image_width_, image_height_
        );
        camera_info_sub_.reset();
    }


    void DropperTask::commandClaw(const std::string & input)
    {
        if (!actuators_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "[DropperTask] actuators_command service is not available."
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
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "[DropperTask] Actuators service reported marker drop failed."
                    );
                }
            }
        );
    }


    void DropperTask::publishZeroWrench()
    {
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }


} // namespace mission

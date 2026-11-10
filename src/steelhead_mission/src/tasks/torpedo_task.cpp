#include "steelhead_mission/tasks/torpedo_task.hpp"

#include <cmath>

using std::placeholders::_1;

namespace mission
{


    TorpedoTask::TorpedoTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      target_detection_label_("circle"),
      image_width_(0.0),
      center_offset_(0.0),
      fire_alignment_threshold_(0.1),
      have_detection_(false),
      left_torpedo_fired_(false)
    {
        detections_sub_ = node_->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
            "yolo_detector/front/detections", 10,
            std::bind(&TorpedoTask::detectionsCallback, this, _1)
        );

        camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "drivers/front_camera/camera_info", 10,
            std::bind(&TorpedoTask::cameraInfoCallback, this, _1)
        );

        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );

        actuators_client_ = node_->create_client<steelhead_interfaces::srv::ActuatorsCommand>(
            "/steelhead/controls/actuators_command"
        );
    }


    BT::PortsList TorpedoTask::providedPorts()
    {
        return {
            BT::InputPort<std::string>("label", "circle", "Target detection class label"),
            BT::InputPort<double>("center_offset", 0.0, "Desired horizontal box position (-1.0 left to 1.0 right)"),
            BT::InputPort<double>("fire_alignment_threshold", 0.1, "Alignment error threshold to fire torpedoes")
        };
    }


    BT::NodeStatus TorpedoTask::onStart()
    {
        target_detection_label_ = "circle";
        center_offset_ = 0.0;
        fire_alignment_threshold_ = 0.1;

        getInput("label", target_detection_label_);
        getInput("center_offset", center_offset_);
        getInput("fire_alignment_threshold", fire_alignment_threshold_);

        have_detection_ = false;
        left_torpedo_fired_ = false;
        latest_detections_.reset();

        RCLCPP_INFO(
            node_->get_logger(),
            "[TorpedoTask] Started (label='%s', center_offset=%.2f, threshold=%.2f)",
            target_detection_label_.c_str(),
            center_offset_,
            fire_alignment_threshold_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus TorpedoTask::onRunning()
    {
        if (image_width_ <= 0.0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[TorpedoTask] Waiting for CameraInfo, holding."
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
                "[TorpedoTask] Detection box '%s' not in sight, advancing slowly...",
                target_detection_label_.c_str()
            );
            return BT::NodeStatus::RUNNING;
        }

        double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
        double horizontal_error = box_center_x - image_width_ / 2.0;
        double normalized_error = horizontal_error / (image_width_ / 2.0);
        double offset_error = normalized_error - center_offset_;

        auto adjust = steelhead_interfaces::msg::HoverAdjustment();
        adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        adjust.input.force.y = -15.0 * offset_error;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[TorpedoTask] Target at center_x=%.1f (offset error=%.2f, target offset=%.2f), swaying to center...",
            box_center_x, offset_error, center_offset_
        );

        hover_adjust_pub_->publish(adjust);

        if (!left_torpedo_fired_ && std::abs(offset_error) < fire_alignment_threshold_)
        {
            fireTorpedo(steelhead_interfaces::srv::ActuatorsCommand::Request::FIRE_LEFT_TORPEDO);
            left_torpedo_fired_ = true;
        }

        if (left_torpedo_fired_ && std::abs(offset_error) < fire_alignment_threshold_)
        {
            fireTorpedo(steelhead_interfaces::srv::ActuatorsCommand::Request::FIRE_RIGHT_TORPEDO);
            publishZeroWrench();
            RCLCPP_INFO(node_->get_logger(), "[TorpedoTask] Fired both torpedoes!");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }


    void TorpedoTask::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[TorpedoTask] Halted; zeroed wrench.");
    }


    void TorpedoTask::detectionsCallback(
        const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
    {
        latest_detections_ = msg;
    }


    void TorpedoTask::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        image_width_ = static_cast<double>(msg->width);
        RCLCPP_INFO(node_->get_logger(), "[TorpedoTask] Got image width from CameraInfo: %.0f px", image_width_);
        camera_info_sub_.reset();
    }


    void TorpedoTask::fireTorpedo(const std::string & input)
    {
        if (!actuators_client_->service_is_ready())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[TorpedoTask] Aligned with target but actuators_command service is not available."
            );
            return;
        }

        auto request = std::make_shared<steelhead_interfaces::srv::ActuatorsCommand::Request>();
        request->input = input;

        RCLCPP_INFO(node_->get_logger(), "[TorpedoTask] Firing torpedo: '%s'", input.c_str());

        actuators_client_->async_send_request(
            request,
            [this](rclcpp::Client<steelhead_interfaces::srv::ActuatorsCommand>::SharedFuture future)
            {
                if (future.get()->succeeded)
                {
                    RCLCPP_INFO(node_->get_logger(), "[TorpedoTask] Torpedo fired successfully.");
                }
                else
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "[TorpedoTask] Actuators service reported torpedo firing failed."
                    );
                }
            }
        );
    }


    void TorpedoTask::publishZeroWrench()
    {
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }


} // namespace mission

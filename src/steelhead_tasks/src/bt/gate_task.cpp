#include "steelhead_tasks/bt/gate_task.hpp"

using std::placeholders::_1;

namespace tasks
{


    GateTask::GateTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      target_detection_label_("sos"),
      image_width_(0.0),
      pass_timeout_(4.0),
      center_offset_(0.1),
      have_gate_(false),
      have_seen_target_(false)
    {
        detections_sub_ = node_->create_subscription<steelhead_interfaces::msg::DetectionBoxArray>(
            "yolo_detector/front/detections", 10,
            std::bind(&GateTask::detectionsCallback, this, _1)
        );

        camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "drivers/front_camera/camera_info", 10,
            std::bind(&GateTask::cameraInfoCallback, this, _1)
        );

        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );
    }


    BT::PortsList GateTask::providedPorts()
    {
        return {
            BT::InputPort<std::string>("label", "sos", "Target detection class label"),
            BT::InputPort<double>("pass_timeout", 4.0, "Seconds to coast forward after losing sight of gate"),
            BT::InputPort<double>("center_offset", 0.1, "Desired horizontal box position (-1.0 left to 1.0 right)")
        };
    }


    BT::NodeStatus GateTask::onStart()
    {
        target_detection_label_ = "sos";
        pass_timeout_ = 4.0;
        center_offset_ = 0.1;

        getInput("label", target_detection_label_);
        getInput("pass_timeout", pass_timeout_);
        getInput("center_offset", center_offset_);

        have_gate_ = false;
        have_seen_target_ = false;
        latest_detections_.reset();

        RCLCPP_INFO(
            node_->get_logger(),
            "[GateTask] Started (label='%s', center_offset=%.2f, pass_timeout=%.1f s)",
            target_detection_label_.c_str(),
            center_offset_,
            pass_timeout_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus GateTask::onRunning()
    {
        if (image_width_ <= 0.0)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[GateTask] Waiting for CameraInfo, holding."
            );
            publishZeroWrench();
            return BT::NodeStatus::RUNNING;
        }

        have_gate_ = false;
        if (latest_detections_)
        {
            float best_confidence = -1.0f;
            for (const auto & box : latest_detections_->boxes)
            {
                if (box.label == target_detection_label_ && box.confidence > best_confidence)
                {
                    best_confidence = box.confidence;
                    latest_detected_box_ = box;
                    have_gate_ = true;
                }
            }
        }

        if (!have_gate_)
        {
            if (!have_seen_target_)
            {
                auto search = steelhead_interfaces::msg::HoverAdjustment();
                search.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
                search.input.force.x = 5.0;
                hover_adjust_pub_->publish(search);
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "[GateTask] Detection box '%s' not in sight, advancing slowly...",
                    target_detection_label_.c_str()
                );
                return BT::NodeStatus::RUNNING;
            }

            double since_lost = (node_->now() - last_detection_time_).seconds();
            if (since_lost >= pass_timeout_)
            {
                publishZeroWrench();
                RCLCPP_INFO(node_->get_logger(), "[GateTask] Passed through gate!");
                return BT::NodeStatus::SUCCESS;
            }

            auto coast = steelhead_interfaces::msg::HoverAdjustment();
            coast.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
            coast.input.force.x = 15.0;
            hover_adjust_pub_->publish(coast);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "[GateTask] Continuing to drift past gate (%.1f / %.1f s)...",
                since_lost, pass_timeout_
            );
            return BT::NodeStatus::RUNNING;
        }

        have_seen_target_ = true;
        last_detection_time_ = node_->now();

        double box_center_x = latest_detected_box_.x + latest_detected_box_.width / 2.0;
        double horizontal_error = box_center_x - image_width_ / 2.0;
        double normalized_error = horizontal_error / (image_width_ / 2.0);
        double offset_error = normalized_error - center_offset_;

        auto adjust = steelhead_interfaces::msg::HoverAdjustment();
        adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        adjust.input.force.x = 5.0;
        adjust.input.force.y = -15.0 * offset_error;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[GateTask] Gate at center_x=%.1f (offset error=%.2f, target offset=%.2f), swaying and driving forward.",
            box_center_x, offset_error, center_offset_
        );

        hover_adjust_pub_->publish(adjust);
        return BT::NodeStatus::RUNNING;
    }


    void GateTask::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[GateTask] Halted; zeroed wrench.");
    }


    void GateTask::detectionsCallback(
        const steelhead_interfaces::msg::DetectionBoxArray::SharedPtr msg)
    {
        latest_detections_ = msg;
    }


    void GateTask::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        image_width_ = static_cast<double>(msg->width);
        RCLCPP_INFO(node_->get_logger(), "[GateTask] Got image width from CameraInfo: %.0f px", image_width_);
        camera_info_sub_.reset();
    }


    void GateTask::publishZeroWrench()
    {
        auto stop = steelhead_interfaces::msg::HoverAdjustment();
        stop.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(stop);
    }


} // namespace tasks

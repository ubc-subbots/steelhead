#include "steelhead_mission/tasks/path_task.hpp"

using std::placeholders::_1;

namespace mission
{


    PathTask::PathTask(
        const std::string & name,
        const BT::NodeConfig & config,
        rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      speed_(5.0),
      sway_gain_(10.0),
      align_tolerance_(0.2),
      lost_timeout_(1.0),
      exit_drive_time_(2.0),
      stage_(Stage::SEARCH),
      fresh_detection_(false),
      aligned_(false)
    {
        detection_sub_ = node_->create_subscription<steelhead_interfaces::msg::PathMarker>(
            "path_marker/detection", 10,
            std::bind(&PathTask::detectionCallback, this, _1)
        );

        hover_adjust_pub_ = node_->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );
    }


    BT::PortsList PathTask::providedPorts()
    {
        return {
            BT::InputPort<double>("speed", 5.0, "Forward force while searching and following"),
            BT::InputPort<double>("sway_gain", 10.0, "Sideways force per unit of lateral offset"),
            BT::InputPort<double>("align_tolerance", 0.2, "Heading error (rad) below which the robot drives forward"),
            BT::InputPort<double>("lost_timeout", 1.0, "Seconds without a detection before the marker counts as gone"),
            BT::InputPort<double>("exit_drive_time", 2.0, "Seconds to keep driving after passing the end of the marker")
        };
    }


    BT::NodeStatus PathTask::onStart()
    {
        speed_ = 5.0;
        sway_gain_ = 10.0;
        align_tolerance_ = 0.2;
        lost_timeout_ = 1.0;
        exit_drive_time_ = 2.0;

        getInput("speed", speed_);
        getInput("sway_gain", sway_gain_);
        getInput("align_tolerance", align_tolerance_);
        getInput("lost_timeout", lost_timeout_);
        getInput("exit_drive_time", exit_drive_time_);

        stage_ = Stage::SEARCH;
        fresh_detection_ = false;
        aligned_ = false;
        last_seen_time_ = node_->now();
        exit_start_time_ = node_->now();

        RCLCPP_INFO(
            node_->get_logger(),
            "[PathTask] Started (speed=%.1f, sway_gain=%.1f, align_tolerance=%.2f rad, "
            "lost_timeout=%.1f s, exit_drive_time=%.1f s)",
            speed_, sway_gain_, align_tolerance_, lost_timeout_, exit_drive_time_
        );

        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus PathTask::onRunning()
    {
        switch (stage_)
        {
            case Stage::SEARCH:
                return search();
            case Stage::FOLLOW:
                return follow();
            case Stage::EXIT:
                return leavePath();
        }
        return BT::NodeStatus::RUNNING;
    }


    void PathTask::onHalted()
    {
        publishZeroWrench();
        RCLCPP_WARN(node_->get_logger(), "[PathTask] Halted; zeroed wrench.");
    }


    void PathTask::detectionCallback(const steelhead_interfaces::msg::PathMarker::SharedPtr msg)
    {
        if (!msg->detected)
        {
            return;
        }
        latest_detection_ = *msg;
        fresh_detection_ = true;
        last_seen_time_ = node_->now();
    }


    BT::NodeStatus PathTask::search()
    {
        if (fresh_detection_ && latest_detection_.centroid_y > FOUND_CENTROID_Y)
        {
            RCLCPP_INFO(node_->get_logger(), "[PathTask] Marker under the camera, following it.");
            stage_ = Stage::FOLLOW;
            return follow();
        }

        publishAdjustment(speed_, 0.0, false, 0.0);
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[PathTask] SEARCH: marker not under the camera yet, advancing..."
        );
        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus PathTask::follow()
    {
        const auto & marker = latest_detection_;

        double since_seen = (node_->now() - last_seen_time_).seconds();
        if (since_seen > lost_timeout_)
        {
            if (aligned_ && marker.centroid_y > EXIT_CENTROID_Y)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "[PathTask] Marker passed behind the robot, driving clear for %.1f s.",
                    exit_drive_time_
                );
                stage_ = Stage::EXIT;
                exit_start_time_ = node_->now();
                return leavePath();
            }

            publishZeroWrench();
            RCLCPP_WARN(
                node_->get_logger(),
                "[PathTask] Lost the marker (last heading error %.2f rad, centroid_y %.2f).",
                marker.heading_error, marker.centroid_y
            );
            return BT::NodeStatus::FAILURE;
        }

        // The marker's direction can only be measured when a long piece of it is in view,
        // which is when its centre is near the middle of the image. Near the top or bottom
        // edge only a rounded end is visible, so keep the current heading and alignment.
        bool reliable = std::abs(marker.centroid_y) < RELIABLE_CENTROID_Y;
        double force_y = 0.0;
        bool change_yaw = false;
        double yaw_offset = 0.0;
        if (reliable)
        {
            aligned_ = std::abs(marker.heading_error) < align_tolerance_;
            force_y = -sway_gain_ * marker.lateral_offset;
            // Only turn on a new detection, so the same camera frame is never applied twice
            change_yaw = fresh_detection_;
            yaw_offset = std::clamp(YAW_GAIN * marker.heading_error, -MAX_YAW_STEP, MAX_YAW_STEP);
        }

        // Drive along the marker once lined up; until then, keep it near the middle of the image
        double force_x = aligned_ ? speed_ : -speed_ * marker.centroid_y;

        publishAdjustment(force_x, force_y, change_yaw, yaw_offset);
        fresh_detection_ = false;

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[PathTask] FOLLOW: heading error %.2f rad, lateral offset %.2f, centroid_y %.2f, %s",
            marker.heading_error, marker.lateral_offset, marker.centroid_y,
            !reliable ? "end of marker in view, holding heading" :
            aligned_ ? "driving along the marker" : "lining up"
        );
        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus PathTask::leavePath()
    {
        double elapsed = (node_->now() - exit_start_time_).seconds();
        if (elapsed >= exit_drive_time_)
        {
            publishZeroWrench();
            RCLCPP_INFO(node_->get_logger(), "[PathTask] Followed the path to its end!");
            return BT::NodeStatus::SUCCESS;
        }

        publishAdjustment(speed_, 0.0, false, 0.0);
        return BT::NodeStatus::RUNNING;
    }


    void PathTask::publishAdjustment(double force_x, double force_y, bool change_yaw, double yaw_offset)
    {
        auto adjust = steelhead_interfaces::msg::HoverAdjustment();
        adjust.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        adjust.input.force.x = force_x;
        adjust.input.force.y = force_y;
        adjust.change_yaw = change_yaw;
        adjust.yaw_offset = yaw_offset;
        hover_adjust_pub_->publish(adjust);
    }


    void PathTask::publishZeroWrench()
    {
        publishAdjustment(0.0, 0.0, false, 0.0);
    }


} // namespace mission

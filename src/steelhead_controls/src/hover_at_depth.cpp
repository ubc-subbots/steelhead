#include "steelhead_controls/hover_at_depth.hpp"

#include <steelhead_interfaces/msg/detail/hover_adjustment__struct.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
using std::placeholders::_1;

namespace steelhead_controls
{

    HoverAtDepth::HoverAtDepth(const rclcpp::NodeOptions &options)
        : Node("hover_at_depth", options)
    {
        this->declare_parameter<float>("depth", 0.5);
        this->get_parameter("depth", hover_depth_);
        if (hover_depth_ < 0.0) hover_depth_ = 0.0;
        RCLCPP_INFO(this->get_logger(), hover_depth_ ? "Hovering at %fm below surface." : "Negative or 0.0 depth provided, only adjusting orientation.", hover_depth_);

        this->declare_parameter<bool>("hold_yaw", false);
        this->get_parameter("hold_yaw", hold_yaw_);
        RCLCPP_INFO(this->get_logger(), hold_yaw_ ? "Adjusting yaw" : "Not adjusting yaw");

        // If no adjustments is published, adjustment_ is zeroed out and nothing is applied
        adjustments_ = std::make_shared<steelhead_interfaces::msg::HoverAdjustment>();

        // Similarly, if we don't adjust for depth, created a default pointer with no error for callback to work
        if (!hover_depth_) pressure_sensor_ = std::make_shared<steelhead_interfaces::msg::PressureSensor>();

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("controls/input_pose", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("drivers/imu/out", 10, std::bind(&HoverAtDepth::imu_callback, this, _1));
        if (hover_depth_) pressure_subscription_ = this->create_subscription<steelhead_interfaces::msg::PressureSensor>("drivers/pressure_sensor", 10, std::bind(&HoverAtDepth::depth_callback, this, _1));
        adjustment_subscription_ = this->create_subscription<steelhead_interfaces::msg::HoverAdjustment>("controls/hover_adjust", 10, std::bind(&HoverAtDepth::adjust_callback, this, _1));
    }

    void HoverAtDepth::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_ = msg;
        publish_error_to_target();
    }

    void HoverAtDepth::depth_callback(const steelhead_interfaces::msg::PressureSensor::SharedPtr msg)
    {
        pressure_sensor_ = msg;
        publish_error_to_target();
    }

    void HoverAtDepth::adjust_callback(const steelhead_interfaces::msg::HoverAdjustment::SharedPtr msg)
    {
        adjustments_ = msg;
    }

    void HoverAtDepth::publish_error_to_target()
    {
        // !TODO replace with a ros2 message filter so we don't poll on callback
        if (adjustments_->type == steelhead_interfaces::msg::HoverAdjustment::FULL) {
            // !TODO: Update to allow the other adjustments
            geometry_msgs::msg::Pose pose;

            tf2::Quaternion error;
            error.setRPY(0.0, adjustments_->input.torque.y, 0.0);

            pose.orientation.x = error.x();
            pose.orientation.y = error.y();
            pose.orientation.z = error.z();
            pose.orientation.w = error.w();

            pose_publisher_->publish(pose);
            return;
        }

        // Publish even if one sensor is not yet available so downstream
        // components (allocator/thrusters) receive commands at startup.
        geometry_msgs::msg::Pose pose;

        // Depth: if we have a pressure sensor, compute depth error as before,
        // otherwise leave depth at 0 (no depth correction until sensor arrives).
        if (hover_depth_) {
            if (pressure_sensor_ != nullptr) {
                pose.position.z = pressure_sensor_->depth - hover_depth_;
            } else {
                pose.position.z = 0.0;
            }
        }

        pose.position.x = adjustments_->input.force.x;
        pose.position.y = adjustments_->input.force.y;
        if (adjustments_->input.force.z) pose.position.z = adjustments_->input.force.z < 0 ? -0.2 : 0.2; // standardize inputs

        // Orientation: if IMU is present use it, otherwise assume identity
        // orientation (0,0,0) so we can still publish a sensible target.
        tf2::Quaternion q_current;
        if (imu_ != nullptr) {
            q_current.setX(imu_->orientation.x);
            q_current.setY(imu_->orientation.y);
            q_current.setZ(imu_->orientation.z);
            q_current.setW(imu_->orientation.w);
        } else {
            q_current.setRPY(0.0, 0.0, 0.0);
        }

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

        double target_yaw = 0.0;
        if (!hold_yaw_) {
            if (adjustments_->input.torque.z) {
                target_yaw = yaw + (adjustments_->input.torque.z < 0 ? -0.01 : 0.01);
            } else {
                target_yaw = yaw;
            }
        }

        tf2::Quaternion q_target;
        q_target.setRPY(0.0, 0.0, target_yaw);

        tf2::Quaternion q_error = q_target * q_current.inverse();
        q_error.normalize();

        pose.orientation.x = q_error.x();
        pose.orientation.y = q_error.y();
        pose.orientation.z = q_error.z();
        pose.orientation.w = q_error.w();

        pose_publisher_->publish(pose);
    }

} // namespace steelhead_controls

int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions();
        rclcpp::spin(std::make_shared<steelhead_controls::HoverAtDepth>(options));
        rclcpp::shutdown();
    }
    catch (rclcpp::exceptions::RCLError const &)
    {
        // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
    } // during testing sometimes throws error
    return 0;
}
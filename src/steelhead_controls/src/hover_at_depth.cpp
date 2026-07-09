#include "steelhead_controls/hover_at_depth.hpp"

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
        RCLCPP_INFO(this->get_logger(), hover_depth_ ? "Hovering at depth=%fm (pressure - depth = z error)." : "depth=0.0; only adjusting orientation.", hover_depth_);

        this->declare_parameter<bool>("adjust_yaw", false);
        this->get_parameter("adjust_yaw", adjust_yaw_);
        RCLCPP_INFO(this->get_logger(), adjust_yaw_ ? "Adjusting yaw" : "Not adjusting yaw");

        adjustments_ = std::make_shared<geometry_msgs::msg::Wrench>();

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("controls/input_pose", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("drivers/imu/out", 10, std::bind(&HoverAtDepth::imu_callback, this, _1));
        if (hover_depth_) pressure_subscription_ = this->create_subscription<steelhead_interfaces::msg::PressureSensor>("drivers/pressure_sensor", 10, std::bind(&HoverAtDepth::depth_callback, this, _1));
        wrench_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>("controls/hover_adjust", 10, std::bind(&HoverAtDepth::wrench_callback, this, _1));
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

    void HoverAtDepth::wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        adjustments_ = msg;
    }

    void HoverAtDepth::publish_error_to_target()
    {
        if (pressure_sensor_ != nullptr && imu_ != nullptr) {
            geometry_msgs::msg::Pose pose;
            if (hover_depth_) pose.position.z = pressure_sensor_->depth - hover_depth_;
            pose.position.x = adjustments_->force.x;
            pose.position.y = adjustments_->force.y;
            if (adjustments_->force.z) pose.position.z = adjustments_->force.z;
        
            tf2::Quaternion q_current(
                imu_->orientation.x,
                imu_->orientation.y,
                imu_->orientation.z,
                imu_->orientation.w
            );

            double roll, pitch, current_yaw;
            tf2::Matrix3x3(q_current).getRPY(roll, pitch, current_yaw);

            double target_yaw = adjust_yaw_ ? 0.0 : current_yaw - adjustments_->torque.z;

            tf2::Quaternion q_error;
            q_error.setRPY(0.0, 0.0, current_yaw - target_yaw);
            q_error.normalize();

            pose.orientation.x = q_error.x();
            pose.orientation.y = q_error.y();
            pose.orientation.z = q_error.z();
            pose.orientation.w = q_error.w();
            
            pose_publisher_->publish(pose);
        }
    }

}

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
    }
    return 0;
}
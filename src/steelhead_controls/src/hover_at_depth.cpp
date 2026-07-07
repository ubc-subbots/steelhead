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
        if (hover_depth_ < 0.0) hover_depth_ = 0.0;
        RCLCPP_INFO(this->get_logger(), hover_depth_ ? "Hovering at %fm below surface." : "Negative or 0.0 depth provided, only adjusting orientation.", hover_depth_);

        this->declare_parameter<bool>("hold_yaw", false);
        this->get_parameter("hold_yaw", hold_yaw_);
        RCLCPP_INFO(this->get_logger(), hold_yaw_ ? "Adjusting yaw" : "Not adjusting yaw");

        // Adjustments expire if the commander stops publishing, so a dropped teleop/pipeline
        // connection can't leave a stale command driving the robot indefinitely
        this->declare_parameter<float>("adjust_timeout", 1.0);
        this->get_parameter("adjust_timeout", adjust_timeout_);
        last_adjust_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

        // If no adjustments are published, adjustments_ is zeroed out and nothing is applied
        adjustments_ = std::make_shared<geometry_msgs::msg::Wrench>();

        // Similarly, if we don't adjust for depth, created a default pointer with no error for callback to work
        if (!hover_depth_) pressure_sensor_ = std::make_shared<steelhead_interfaces::msg::PressureSensor>();

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
        last_adjust_time_ = std::chrono::steady_clock::now();
    }

    void HoverAtDepth::publish_error_to_target()
    {
        // !TODO replace with a ros2 message filter so we don't poll on callback
        if (pressure_sensor_ != nullptr && imu_ != nullptr) {
            geometry_msgs::msg::Wrench adjustments = *adjustments_;
            if (adjust_timeout_ > 0.0f &&
                std::chrono::duration<float>(std::chrono::steady_clock::now() - last_adjust_time_).count() > adjust_timeout_) {
                adjustments = geometry_msgs::msg::Wrench();
            }

            geometry_msgs::msg::Pose pose;
            if (hover_depth_) pose.position.z = pressure_sensor_->depth - hover_depth_;
            pose.position.x = adjustments.force.x;
            pose.position.y = adjustments.force.y;
            if (adjustments.force.z) pose.position.z = adjustments.force.z < 0 ? -0.2 : 0.2; // standardize inputs
        
            tf2::Quaternion q_current(
                imu_->orientation.x,
                imu_->orientation.y,
                imu_->orientation.z,
                imu_->orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

            double target_yaw = 0.0;
            if (!hold_yaw_) {
                if (adjustments.torque.z) {
                    target_yaw = yaw + (adjustments.torque.z < 0 ? -0.01 : 0.01);
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
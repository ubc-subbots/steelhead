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

        this->declare_parameter<double>("yaw_step", 0.01);
        this->declare_parameter<double>("sensor_timeout", 0.5);

        // If no adjustments are published, adjustments_ is zeroed out and nothing is applied
        adjustments_ = std::make_shared<geometry_msgs::msg::Wrench>();

        // Similarly, if we don't adjust for depth, created a default pointer with no error for callback to work
        if (!hover_depth_) pressure_sensor_ = std::make_shared<steelhead_interfaces::msg::PressureSensor>();

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("controls/input_pose", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("drivers/imu/out", 10, std::bind(&HoverAtDepth::imu_callback, this, _1));
        if (hover_depth_) pressure_subscription_ = this->create_subscription<steelhead_interfaces::msg::PressureSensor>("drivers/pressure_sensor", 10, std::bind(&HoverAtDepth::depth_callback, this, _1));
        wrench_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>("controls/hover_adjust", 10, std::bind(&HoverAtDepth::wrench_callback, this, _1));

        last_imu_time_ = std::chrono::steady_clock::now();
        last_pressure_time_ = last_imu_time_;
        watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HoverAtDepth::watchdog_callback, this));
    }

    void HoverAtDepth::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_ = msg;
        last_imu_time_ = std::chrono::steady_clock::now();
        if (!sensors_stale_) publish_error_to_target();
    }

    void HoverAtDepth::depth_callback(const steelhead_interfaces::msg::PressureSensor::SharedPtr msg)
    {
        pressure_sensor_ = msg;
        last_pressure_time_ = std::chrono::steady_clock::now();
        if (!sensors_stale_) publish_error_to_target();
    }

    void HoverAtDepth::wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        adjustments_ = msg;
    }

    void HoverAtDepth::watchdog_callback()
    {
        // Before the first IMU message the PID receives no input at all, which is already safe
        if (imu_ == nullptr) return;

        double timeout = this->get_parameter("sensor_timeout").as_double();
        auto now = std::chrono::steady_clock::now();
        double age = std::chrono::duration<double>(now - last_imu_time_).count();
        if (hover_depth_) {
            double pressure_age = std::chrono::duration<double>(now - last_pressure_time_).count();
            if (pressure_age > age) age = pressure_age;
        }

        if (age > timeout) {
            if (!sensors_stale_) {
                sensors_stale_ = true;
                RCLCPP_WARN(this->get_logger(), "Sensor data stale for %.2fs, zeroing output error (thrusters idle).", age);
            }
            geometry_msgs::msg::Pose zero;
            zero.orientation.w = 1.0;
            pose_publisher_->publish(zero);
        } else if (sensors_stale_) {
            sensors_stale_ = false;
            RCLCPP_INFO(this->get_logger(), "Sensor data recovered, resuming control.");
        }
    }

    void HoverAtDepth::publish_error_to_target()
    {
        // !TODO replace with a ros2 message filter so we don't poll on callback
        if (pressure_sensor_ != nullptr && imu_ != nullptr) {
            geometry_msgs::msg::Pose pose;
            if (hover_depth_) pose.position.z = pressure_sensor_->depth - hover_depth_;
            pose.position.x = adjustments_->force.x;
            pose.position.y = adjustments_->force.y;
            if (adjustments_->force.z) pose.position.z = adjustments_->force.z < 0 ? -0.2 : 0.2; // standardize inputs
        
            tf2::Quaternion q_current(
                imu_->orientation.x,
                imu_->orientation.y,
                imu_->orientation.z,
                imu_->orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

            double target_yaw = 0.0;
            if (adjustments_->torque.z) {
                double yaw_step = this->get_parameter("yaw_step").as_double();
                target_yaw = yaw + (adjustments_->torque.z < 0 ? -yaw_step : yaw_step);
            } else if (!hold_yaw_) {
                target_yaw = yaw;
            }
            
            
            tf2::Quaternion q_target;
            q_target.setRPY(0.0, 0.0, target_yaw);

            tf2::Quaternion q_error = q_current.inverse() * q_target;
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
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
        RCLCPP_INFO(this->get_logger(), "Hovering at %fm below surface", hover_depth_);

        this->declare_parameter<bool>("adjust_yaw", false);
        this->get_parameter("adjust_yaw", adjust_yaw_);
        RCLCPP_INFO(this->get_logger(), adjust_yaw_ ? "Adjusting yaw" : "Not adjusting yaw");

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("controls/input_pose", 10);
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("drivers/imu/out", 10, std::bind(&HoverAtDepth::imu_callback, this, _1));
        pressure_subscription_ = this->create_subscription<steelhead_interfaces::msg::PressureSensor>("drivers/pressure_sensor", 10, std::bind(&HoverAtDepth::depth_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Hover node successfully started!");
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

    void HoverAtDepth::publish_error_to_target()
    {
        // !TODO replace with a ros2 message filter so we don't poll on callback
        if (pressure_sensor_ != nullptr && imu_ != nullptr) {
            geometry_msgs::msg::Pose pose;
            pose.position.z = pressure_sensor_->depth - hover_depth_;
        
            if (adjust_yaw_) {
                pose.orientation.w = imu_->orientation.w;
                pose.orientation.x = -imu_->orientation.x;
                pose.orientation.y = -imu_->orientation.y;
                pose.orientation.z = -imu_->orientation.z;
            } else {
                tf2::Quaternion q_current(
                    imu_->orientation.x,
                    imu_->orientation.y,
                    imu_->orientation.z,
                    imu_->orientation.w
                );
                
                double roll, pitch, yaw;
                tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
                
                tf2::Quaternion q_target;
                q_target.setRPY(0.0, 0.0, yaw);
                
                tf2::Quaternion q_error = q_target * q_current.inverse();
                q_error.normalize();
                
                pose.orientation.w = q_error.getW();
                pose.orientation.x = q_error.getX();
                pose.orientation.y = q_error.getY();
                pose.orientation.z = q_error.getZ();
            }
            
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
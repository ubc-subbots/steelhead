#include "steelhead_controls/hover_at_depth.hpp"

#include <steelhead_interfaces/msg/detail/hover_adjustment__struct.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
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

        this->declare_parameter<std::string>("base_frame", "base_link");
        this->get_parameter("base_frame", base_frame_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // If no adjustments is published, adjustment_ is zeroed out and nothing is applied
        adjustments_ = std::make_shared<steelhead_interfaces::msg::HoverAdjustment>();

        // Similarly, if we don't adjust for depth, created a default pointer with no error for callback to work
        if (!hover_depth_) pressure_sensor_ = std::make_shared<steelhead_interfaces::msg::PressureSensor>();

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("controls/input_pose", rclcpp::QoS(10));
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("drivers/imu/out", rclcpp::QoS(10), std::bind(&HoverAtDepth::imu_callback, this, _1));
        if (hover_depth_) pressure_subscription_ = this->create_subscription<steelhead_interfaces::msg::PressureSensor>("drivers/pressure_sensor", rclcpp::QoS(10), std::bind(&HoverAtDepth::depth_callback, this, _1));
        adjustment_subscription_ = this->create_subscription<steelhead_interfaces::msg::HoverAdjustment>("controls/hover_adjust", rclcpp::QoS(10), std::bind(&HoverAtDepth::adjust_callback, this, _1));
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
        if (pressure_sensor_ != nullptr && imu_ != nullptr) {
            if (adjustments_->type == steelhead_interfaces::msg::HoverAdjustment::PARTIAL) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = adjustments_->input.force.x;
                pose.position.y = adjustments_->input.force.y;
                // add a field for adjustments for full or partial
            
                tf2::Quaternion q_sensor(
                    imu_->orientation.x,
                    imu_->orientation.y,
                    imu_->orientation.z,
                    imu_->orientation.w
                );
                tf2::Quaternion q_current = q_sensor;
                std::string imu_tf_err;
                if (!imu_->header.frame_id.empty() && tf_buffer_->canTransform(base_frame_, imu_->header.frame_id, tf2::TimePointZero, &imu_tf_err)) {
                    try {
                        auto tf = tf_buffer_->lookupTransform(base_frame_, imu_->header.frame_id, tf2::TimePointZero);
                        tf2::Quaternion q_base_sensor(
                            tf.transform.rotation.x,
                            tf.transform.rotation.y,
                            tf.transform.rotation.z,
                            tf.transform.rotation.w
                        );
                        // world_R_base = world_R_sensor * (base_R_sensor)^-1
                        q_current = (q_sensor * q_base_sensor.inverse()).normalized();
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_WARN_ONCE(this->get_logger(), "Could not transform IMU orientation to %s: %s", base_frame_.c_str(), ex.what());
                    }
                } else if (!imu_->header.frame_id.empty()) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for transform from %s to %s: %s", imu_->header.frame_id.c_str(), base_frame_.c_str(), imu_tf_err.c_str());
                }

                double roll, pitch, yaw;
                tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);

                float current_depth = pressure_sensor_->depth;
                std::string depth_tf_err;
                if (hover_depth_ && !pressure_sensor_->header.frame_id.empty() && tf_buffer_->canTransform(base_frame_, pressure_sensor_->header.frame_id, tf2::TimePointZero, &depth_tf_err)) {
                    try {
                        auto tf = tf_buffer_->lookupTransform(base_frame_, pressure_sensor_->header.frame_id, tf2::TimePointZero);
                        tf2::Vector3 sensor_offset_base(
                            tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z
                        );
                        tf2::Vector3 sensor_offset_world = tf2::quatRotate(q_current, sensor_offset_base);
                        current_depth = static_cast<float>(pressure_sensor_->depth + sensor_offset_world.z());
                    } catch (const tf2::TransformException & ex) {
                        RCLCPP_WARN_ONCE(this->get_logger(), "Could not transform depth sensor offset: %s", ex.what());
                    }
                } else if (hover_depth_ && !pressure_sensor_->header.frame_id.empty()) {
                    RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for transform from %s to %s: %s", pressure_sensor_->header.frame_id.c_str(), base_frame_.c_str(), depth_tf_err.c_str());
                }
                if (hover_depth_) pose.position.z = current_depth - hover_depth_;
                if (adjustments_->input.force.z) pose.position.z = adjustments_->input.force.z < 0 ? -0.2 : 0.2; // standardize inputs

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
            } else {
                // !TODO: Update to allow the other adjustments
                geometry_msgs::msg::Pose pose;

                tf2::Quaternion error;
                error.setRPY(0.0, adjustments_->input.torque.y, 0.0);

                pose.orientation.x = error.x();
                pose.orientation.y = error.y();
                pose.orientation.z = error.z();
                pose.orientation.w = error.w();

                pose_publisher_->publish(pose); 
            }
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
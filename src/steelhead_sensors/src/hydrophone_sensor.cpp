#include "steelhead_sensors/hydrophone_sensor.hpp"

#include "steelhead_interfaces/msg/hydrophone_feedback.hpp"
#include "steelhead_interfaces/msg/pinger_angle.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <numbers>


using std::placeholders::_1;

namespace steelhead_sensors
{

// Constructor with readonly references options
HydrophoneSensor::HydrophoneSensor(const rclcpp::NodeOptions & options)
: Node("hydrophone_sensor", options) 
{
    gz_node_.Subscribe(
        "/world/competition/pose/info",
        &HydrophoneSensor::pinger_callback,
        this
    );

    // Create subscriber to robot poses
    robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/steelhead/state", 10, std::bind(&HydrophoneSensor::robot_callback, this, _1));
    
    publisher_ = this->create_publisher<steelhead_interfaces::msg::HydrophoneFeedback>("sensors/hydrophone_sensor/angle", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Hydrophone Sensor succesfully started!");
}

void HydrophoneSensor::robot_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    auto orientation = msg->pose.pose.orientation;

    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    );

    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw_);

    robot_received_ = true;

    if (pinger_received_) {
        calculate_relative_position();
    }
}


void HydrophoneSensor::pinger_callback(const gz::msgs::Pose_V & msg)
{
    // Clear the dictionary of pingers before updating it with new data
    pingers_.clear();
    pinger_received_ = false;

    // Name is provided for future reference?
    for (const auto & pose : msg.pose()) {
        if (pose.name().starts_with("competition_pinger")) {
            pingers_[pose.name()] = {
                pose.position().x(),
                pose.position().y()
            };
        }
    }

    if (!pingers_.empty()) {
        pinger_received_ = true;
    }

    if (robot_received_ && pinger_received_) {
        calculate_relative_position();
    }
}


void HydrophoneSensor::calculate_relative_position()
{
    auto feedback_msg = steelhead_interfaces::msg::HydrophoneFeedback();


    for (const auto & [name, position] : pingers_) {
        steelhead_interfaces::msg::PingerAngle pinger;

        double dx = position.x - robot_x_;
        double dy = position.y - robot_y_;

        // Distance
        // double distance = std::sqrt(dx * dx + dy * dy);

        double angle = std::atan2(dy, dx) - robot_yaw_;

        pinger.name = name;
        pinger.angle = angle;

        feedback_msg.pingers.push_back(pinger);
    }

    publisher_->publish(feedback_msg);
}

    
} // namespace steelhead_sensors

int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions();
        rclcpp::spin(std::make_shared<steelhead_sensors::HydrophoneSensor>(options));
        rclcpp::shutdown();
    }
    catch (rclcpp::exceptions::RCLError const &)
    {
        // RCLCPP_INFO(this->get_logger(), "Error thrown in main");
    } // during testing sometimes throws error
    return 0;
}
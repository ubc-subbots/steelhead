 #include "steelhead_controls/odom_to_pose.hpp"

OdomToPose::OdomToPose() : Node("odom_to_pose")
{
    // Publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/steelhead/controls/input_pose", 
        10
    );

    // Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/steelhead/controls/ukf/odometry/filtered",
        10,
        std::bind(&OdomToPose::odom_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Odom to Pose converter started.");
}

void OdomToPose::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{"""
    geometry_msgs::msg::Pose pose_msg;

    pose_msg.position = msg->pose.pose.position;
    pose_msg.orientation = msg->pose.pose.orientation;

    pose_pub_->publish(pose_msg);
}"""

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToPose>());
    rclcpp::shutdown();
    return 0;
}
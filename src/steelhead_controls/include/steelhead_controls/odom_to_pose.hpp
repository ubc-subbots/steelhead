#ifndef ODOM_TO_POSE_HPP
#define ODOM_TO_POSE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

class OdomToPose : public rclcpp::Node
{
public:
    OdomToPose();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

#endif

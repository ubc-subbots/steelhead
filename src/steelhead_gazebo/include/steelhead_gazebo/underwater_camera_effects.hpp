#ifndef STEELHEAD_GAZEBO__UNDERWATER_CAMERA_EFFECTS_HPP
#define STEELHEAD_GAZEBO__UNDERWATER_CAMERA_EFFECTS_HPP

#include <memory>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace steelhead_gazebo
{

class UnderwaterCameraEffects : public rclcpp::Node
{
public:
    explicit UnderwaterCameraEffects(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    cv::Mat applyUnderwaterEffects(const cv::Mat& rgb_image, const cv::Mat& depth);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_pub_;

    // Parameters
    double water_color_r_;
    double water_color_g_;
    double water_color_b_;
    double attenuation_coefficient_;
    double blur_strength_;
    double max_depth_;

    cv::Scalar light_water_color_;
    cv::Scalar dark_water_color_;
    cv::Scalar upper_tint_;
    cv::Scalar lower_tint_;
    cv::Scalar gray_weights_;

    cv::Mat latest_depth_;
    std::mutex depth_mutex_;
};

} 

#endif

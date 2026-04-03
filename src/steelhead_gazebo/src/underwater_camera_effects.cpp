#include "steelhead_gazebo/underwater_camera_effects.hpp"
#include <cmath>

namespace steelhead_gazebo
{

UnderwaterCameraEffects::UnderwaterCameraEffects(const rclcpp::NodeOptions & options)
: Node("underwater_camera_effects", options)
{
    this->declare_parameter("water_color_r", 0.15);
    this->declare_parameter("water_color_g", 0.35);
    this->declare_parameter("water_color_b", 0.48);
    this->declare_parameter("attenuation_coefficient", 0.30);
    this->declare_parameter("blur_strength", 2.2);
    this->declare_parameter("max_depth", 12.0);

    water_color_r_ = this->get_parameter("water_color_r").as_double();
    water_color_g_ = this->get_parameter("water_color_g").as_double();
    water_color_b_ = this->get_parameter("water_color_b").as_double();
    attenuation_coefficient_ = this->get_parameter("attenuation_coefficient").as_double();
    blur_strength_ = this->get_parameter("blur_strength").as_double();
    max_depth_ = this->get_parameter("max_depth").as_double();

    // underwater color
    light_water_color_ = cv::Scalar(
        water_color_b_ * 3.2,
        water_color_g_ * 2.5,
        water_color_r_ * 1.5
    );

    dark_water_color_ = cv::Scalar(
        water_color_b_ * 1.1,
        water_color_g_ * 0.65,
        water_color_r_ * 0.25
    );

    upper_tint_ = cv::Scalar(1.28, 1.15, 1.0);
    lower_tint_ = cv::Scalar(1.18, 1.08, 0.92);

    gray_weights_ = cv::Scalar(0.114, 0.587, 0.299);

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/steelhead/drivers/front_camera/image_raw_source",
        10,
        std::bind(&UnderwaterCameraEffects::rgbCallback, this, std::placeholders::_1)
    );

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/steelhead/drivers/front_camera/depth/image_raw",
        10,
        std::bind(&UnderwaterCameraEffects::depthCallback, this, std::placeholders::_1)
    );

    processed_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/steelhead/drivers/front_camera/image_raw",
        10
    );

    RCLCPP_INFO(this->get_logger(), "Underwater camera effects node started (C++)");
    RCLCPP_INFO(this->get_logger(), "Water color (RGB): (%.2f, %.2f, %.2f)",
                water_color_r_, water_color_g_, water_color_b_);
    RCLCPP_INFO(this->get_logger(), "Attenuation: %.2f, Blur: %.2f",
                attenuation_coefficient_, blur_strength_);
}

void UnderwaterCameraEffects::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_depth_ = depth_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting depth image: %s", e.what());
    }
}

void UnderwaterCameraEffects::rgbCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_image = rgb_ptr->image;

        cv::Mat depth;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (latest_depth_.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "No depth data yet, using simulated depth gradient");

                depth = cv::Mat(rgb_image.rows, rgb_image.cols, CV_32FC1);
                for (int i = 0; i < rgb_image.rows; i++) {
                    float depth_val = (float)i / rgb_image.rows * max_depth_;
                    depth.row(i).setTo(depth_val);
                }
            } else {
                depth = latest_depth_.clone();
            }
        }

        if (rgb_image.size() != depth.size()) {
            cv::resize(depth, depth, rgb_image.size());
        }

        cv::Mat processed = applyUnderwaterEffects(rgb_image, depth);

        cv_bridge::CvImage processed_msg;
        processed_msg.header = msg->header;
        processed_msg.encoding = sensor_msgs::image_encodings::BGR8;
        processed_msg.image = processed;

        processed_pub_->publish(*processed_msg.toImageMsg());

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
}

cv::Mat UnderwaterCameraEffects::applyUnderwaterEffects(const cv::Mat& rgb_image,
                                                         const cv::Mat& depth)
{
    const int h = rgb_image.rows;
    const int w = rgb_image.cols;

    cv::Mat image;
    rgb_image.convertTo(image, CV_32FC3, 1.0 / 255.0);

    cv::Mat depth_normalized;
    cv::patchNaNs(depth, 0.0);
    cv::min(depth, max_depth_, depth_normalized);
    cv::max(depth_normalized, 0.0, depth_normalized);
    depth_normalized /= max_depth_;
    cv::pow(depth_normalized, 0.5, depth_normalized);

    cv::Mat attenuation_map;
    cv::exp(-attenuation_coefficient_ * depth_normalized * 3.8f, attenuation_map);

    cv::Mat haze(h, w, CV_32FC1);
    cv::randn(haze, 0.0, 0.003);
    haze = haze.mul(depth_normalized) * 0.5f;
    cv::max(haze, 0.0f, haze);

    const float light_b = static_cast<float>(light_water_color_[0]);
    const float light_g = static_cast<float>(light_water_color_[1]);
    const float light_r = static_cast<float>(light_water_color_[2]);
    const float dark_b = static_cast<float>(dark_water_color_[0]);
    const float dark_g = static_cast<float>(dark_water_color_[1]);
    const float dark_r = static_cast<float>(dark_water_color_[2]);
    const float upper_b = static_cast<float>(upper_tint_[0]);
    const float upper_g = static_cast<float>(upper_tint_[1]);
    const float upper_r = static_cast<float>(upper_tint_[2]);
    const float lower_b = static_cast<float>(lower_tint_[0]);
    const float lower_g = static_cast<float>(lower_tint_[1]);
    const float lower_r = static_cast<float>(lower_tint_[2]);
    const float gray_b = static_cast<float>(gray_weights_[0]);
    const float gray_g = static_cast<float>(gray_weights_[1]);
    const float gray_r = static_cast<float>(gray_weights_[2]);
    const float inv_h = 1.0f / h;

    // Water color, attenuation, tint, and haze
    cv::parallel_for_(cv::Range(0, h), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; y++) {
            const float vert_grad = 1.0f - (float)y * inv_h * 0.7f;
            const float* depth_row = depth_normalized.ptr<float>(y);
            const float* atten_row = attenuation_map.ptr<float>(y);
            const float* haze_row = haze.ptr<float>(y);
            cv::Vec3f* img_row = image.ptr<cv::Vec3f>(y);

            const float tint_b = upper_b * vert_grad + lower_b * (1.0f - vert_grad);
            const float tint_g = upper_g * vert_grad + lower_g * (1.0f - vert_grad);
            const float tint_r = upper_r * vert_grad + lower_r * (1.0f - vert_grad);

            for (int x = 0; x < w; x++) {
                const float d = depth_row[x];
                const float a = atten_row[x];
                const float inv_a = 1.0f - a;
                const float hz = haze_row[x];

                const float wc_b = light_b * (1.0f - d) + dark_b * d;
                const float wc_g = light_g * (1.0f - d) + dark_g * d;
                const float wc_r = light_r * (1.0f - d) + dark_r * d;

                cv::Vec3f& px = img_row[x];
                px[0] = (px[0] * a + wc_b * inv_a) * tint_b + hz;
                px[1] = (px[1] * a + wc_g * inv_a) * tint_g + hz;
                px[2] = (px[2] * a + wc_r * inv_a) * tint_r + hz;
            }
        }
    });

    // Depth-based blur
    if (blur_strength_ > 0) {
        cv::Mat blurred;
        cv::GaussianBlur(image, blurred, cv::Size(11, 11), blur_strength_ * 1.1);

        cv::parallel_for_(cv::Range(0, h), [&](const cv::Range& range) {
            for (int y = range.start; y < range.end; y++) {
                const float* depth_row = depth_normalized.ptr<float>(y);
                cv::Vec3f* img_row = image.ptr<cv::Vec3f>(y);
                const cv::Vec3f* blur_row = blurred.ptr<cv::Vec3f>(y);

                for (int x = 0; x < w; x++) {
                    const float blur_factor = depth_row[x] * 0.45f;
                    const float sharp_factor = 1.0f - blur_factor;
                    img_row[x] = img_row[x] * sharp_factor + blur_row[x] * blur_factor;
                }
            }
        });
    }

    // Brightness, contrast, and desaturation
    cv::parallel_for_(cv::Range(0, h), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; y++) {
            const float vert_grad = 1.0f - (float)y * inv_h * 0.7f;
            const float* depth_row = depth_normalized.ptr<float>(y);
            cv::Vec3f* img_row = image.ptr<cv::Vec3f>(y);

            for (int x = 0; x < w; x++) {
                const float d = depth_row[x];
                cv::Vec3f& px = img_row[x];

                float brightness = (1.85f * vert_grad + 1.05f * (1.0f - vert_grad)) - (d * 0.40f);
                brightness = std::max(0.85f, std::min(2.0f, brightness));
                px *= brightness;

                float contrast = std::max(0.7f, std::min(1.0f, 1.0f - d * 0.10f));
                float contrast_offset = (1.0f - contrast) * 0.5f;
                px = px * contrast + cv::Vec3f(contrast_offset, contrast_offset, contrast_offset);

                float gray_val = px[0] * gray_b + px[1] * gray_g + px[2] * gray_r;
                float desat = 0.12f + d * 0.35f;
                float keep = 1.0f - desat;
                px[0] = px[0] * keep + gray_val * desat;
                px[1] = px[1] * keep + gray_val * desat;
                px[2] = px[2] * keep + gray_val * desat;
            }
        }
    });

    cv::Mat result;
    image.convertTo(result, CV_8UC3, 255.0);

    return result;
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<steelhead_gazebo::UnderwaterCameraEffects>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

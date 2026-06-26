#ifndef STEELHEAD_SLALOMS__SLALOM_DETECTOR
#define STEELHEAD_SLALOMS__SLALOM_DETECTOR

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "steelhead_vision_utils/object_detector.hpp"
#include "steelhead_interfaces/msg/object_offset.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace steelhead_slaloms
{

class SlalomDetector : public rclcpp::Node, public steelhead_vision_utils::ObjectDetector
{
public:
  SlalomDetector(const rclcpp::NodeOptions & options);

  /**
   * Detects red slalom poles in a raw image and publishes their estimated pose offset.
   *
   * @param msg Raw image message from the front camera
   */
  void detect(const sensor_msgs::msg::Image & msg);

private:
  /** Callback used by subscriber to process and publish results */
  void subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * Segment the image using a red HSV color mask.
   * Red wraps around 0/180 in HSV so two inRange calls are OR'd together.
   *
   * @param src A raw BGR image
   * @return A binary segmented image where red regions are white
   */
  cv::Mat segment(cv::Mat & src);

  /**
   * Find the two largest vertically-oriented hulls (the poles) in the segmented image
   * and compute the gap midpoint between them.
   *
   * @param hulls  Convex hulls from the segmented image
   * @param src    The raw image (drawn on if debug is on)
   */
  void detectPolesAndPublish(std::vector<std::vector<cv::Point>> hulls, cv::Mat & src);

  /** Helper for publishing a debug image */
  void debugPublish(cv::Mat & src, image_transport::Publisher & publisher);

  bool debug_ = false;

  image_transport::Subscriber subscription_;
  image_transport::Publisher debug_detection_publisher_;
  image_transport::Publisher debug_segment_publisher_;

  // Estimated pose of the gap between the two nearest poles, relative to AUV
  rclcpp::Publisher<steelhead_interfaces::msg::ObjectOffset>::SharedPtr pole_pose_publisher_;

  // Debug: pose only (for RViz)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pole_pose_only_publisher_;

  // Pixel offset of gap center from image center
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pole_offset_publisher_;
};

}  // namespace steelhead_slaloms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_slaloms::SlalomDetector)

#endif  // STEELHEAD_SLALOMS__SLALOM_DETECTOR
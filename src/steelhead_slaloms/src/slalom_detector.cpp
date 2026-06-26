#include "../include/slalom_detector.hpp"

using namespace std;
using namespace cv;
using std::placeholders::_1;

namespace steelhead_slaloms
{

SlalomDetector::SlalomDetector(const rclcpp::NodeOptions & options)
: Node("slalom_detector", options), ObjectDetector(1.0, false, 400.0)
{
  // ObjectDetector is constructed with debug=false initially; we update after reading the param
  this->declare_parameter("debug", debug_);
  this->get_parameter("debug", debug_);

  if (debug_)
    RCLCPP_INFO(this->get_logger(), "DEBUG ON");
  else
    RCLCPP_INFO(this->get_logger(), "DEBUG OFF");

  rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;

  subscription_ = image_transport::create_subscription(
    this,
    "/steelhead/drivers/front_camera/image_raw",
    bind(&SlalomDetector::subscriberCallback, this, _1),
    "raw",
    sensor_qos_profile
  );

  debug_detection_publisher_ = image_transport::create_publisher(this, "detector/debug/detection");
  debug_segment_publisher_   = image_transport::create_publisher(this, "detector/debug/segment");

  pole_pose_publisher_ = this->create_publisher<steelhead_interfaces::msg::ObjectOffset>(
    "detector/pole_pose", 10);

  if (debug_)
    pole_pose_only_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "detector/pole_pose_only", 10);

  pole_offset_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "detector/pole_offset", 10);

  RCLCPP_INFO(this->get_logger(), "SlalomDetector successfully started!");
}

void SlalomDetector::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  detect(*msg);
}

void SlalomDetector::detect(const sensor_msgs::msg::Image & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat detection = cv_ptr->image;
  cv::Mat segmented = morphological(segment(detection), Size(3, 3), Size(3, 3));
  std::vector<std::vector<cv::Point>> hulls = convexHulls(segmented, detection.rows * detection.cols, 0);
  detectPolesAndPublish(hulls, detection);

  if (debug_) {
    debugPublish(detection, debug_detection_publisher_);
    debugPublish(segmented, debug_segment_publisher_);
  }
}

cv::Mat SlalomDetector::segment(cv::Mat & src)
{
  Mat hsv;
  cvtColor(src, hsv, COLOR_BGR2HSV);

  // Red wraps around 0 and 180 in HSV — need two inRange calls
  Mat upper_hue_mask, lower_hue_mask, color_mask;
  inRange(hsv, Scalar(160, 60, 60), Scalar(180, 255, 255), upper_hue_mask);
  inRange(hsv, Scalar(0,   60, 60), Scalar(10,  255, 255), lower_hue_mask);
  bitwise_or(upper_hue_mask, lower_hue_mask, color_mask);

  return color_mask;
}

void SlalomDetector::detectPolesAndPublish(std::vector<std::vector<Point>> hulls, cv::Mat & src)
{
  if (hulls.size() < 1) {
    return;
  }

  // Find the two largest vertically-oriented hulls — same logic as GateDetector::boundGateUsingPoles
  vector<Point> largest, second_largest;
  double largest_area = 0, second_largest_area = 0;

  for (auto & hull : hulls) {
    float aspect_ratio = 1.0f;
    if (hull.size() >= 3) {
      Rect r = boundingRect(hull);
      aspect_ratio = (float)r.width / r.height;
    }
    // Only consider hulls that are taller than wide (i.e. vertical poles)
    if (aspect_ratio < 1.0f) {
      double area = contourArea(hull);
      if (area > largest_area) {
        second_largest = largest;
        second_largest_area = largest_area;
        largest = hull;
        largest_area = area;
      } else if (area > second_largest_area) {
        second_largest = hull;
        second_largest_area = area;
      }
    }
  }

  // We need at least one pole to do anything useful
  if (largest.empty()) {
    return;
  }

  int width  = src.cols;
  int height = src.rows;

  // Compute bounding box centres for each detected pole
  Rect largest_rect = boundingRect(largest);
  int pole1_cx = largest_rect.x + largest_rect.width / 2;
  int pole1_cy = largest_rect.y + largest_rect.height / 2;

  // Gap centre: if we have two poles use midpoint between them,
  // otherwise assume the gap is to the side of the single visible pole
  int gap_cx, gap_cy;

  if (!second_largest.empty()) {
    Rect second_rect = boundingRect(second_largest);
    int pole2_cx = second_rect.x + second_rect.width / 2;
    int pole2_cy = second_rect.y + second_rect.height / 2;
    gap_cx = (pole1_cx + pole2_cx) / 2;
    gap_cy = (pole1_cy + pole2_cy) / 2;

    if (debug_) {
      rectangle(src, second_rect, Scalar(255, 128, 0), 2);
      circle(src, Point(pole2_cx, pole2_cy), 6, Scalar(255, 128, 0), 3);
    }
  } else {
    // Single pole visible: target the centre of the frame laterally,
    // same depth as the pole
    gap_cx = width / 2;
    gap_cy = pole1_cy;
  }

  // --- Distance estimate (same pinhole model as GateDetector) ---
  // Calibrated against a standard slalom pole width (~0.05 m) at 1 m
  // These values should be tuned once you have real camera data
  double standard_pixel_width = 20.0;   // px width of a pole at standard_distance
  double standard_distance    = 1.0;    // metres
  double standard_width       = 0.05;   // metres (pole diameter)

  double focal_length    = (standard_pixel_width * standard_distance) / standard_width;
  double curr_pixel_width = std::max((double)largest_rect.width, 1.0);
  double distance         = (standard_width * focal_length) / curr_pixel_width;

  // Lateral / vertical offset of gap from image centre
  int frame_cx = width  / 2;
  int frame_cy = height / 2;
  int offset_x = gap_cx - frame_cx;
  int offset_y = gap_cy - frame_cy;

  double distance_y = (double)offset_x / curr_pixel_width * distance;  // lateral  (ENU y)
  double distance_z = (double)offset_y / curr_pixel_width * distance;  // vertical (ENU z)

  // --- Publish ObjectOffset (same message type as gate) ---
  steelhead_interfaces::msg::ObjectOffset pole_pose;
  pole_pose.class_id          = 2;           // arbitrary id distinct from gate (1)
  pole_pose.pose.position.x   = distance;
  pole_pose.pose.position.y   = -distance_y;
  pole_pose.pose.position.z   = -distance_z;
  // No yaw estimate yet — identity quaternion
  pole_pose.pose.orientation.x = 0.0;
  pole_pose.pose.orientation.y = 0.0;
  pole_pose.pose.orientation.z = 0.0;
  pole_pose.pose.orientation.w = 1.0;
  pole_pose_publisher_->publish(pole_pose);

  // --- Publish pixel offset ---
  auto offset_msg = std_msgs::msg::Float32MultiArray();
  offset_msg.data = {static_cast<float>(offset_x), static_cast<float>(offset_y)};
  pole_offset_publisher_->publish(offset_msg);

  // --- Debug drawing ---
  if (debug_) {
    rectangle(src, largest_rect, Scalar(0, 128, 255), 2);
    circle(src, Point(pole1_cx, pole1_cy), 6, Scalar(0, 128, 255), 3);

    Point gap_pt(gap_cx, gap_cy);
    Point frame_pt(frame_cx, frame_cy);
    circle(src, gap_pt,   12, Scalar(0, 255, 0),   6);
    circle(src, frame_pt, 12, Scalar(0, 0,   255),  6);
    line(src, frame_pt, gap_pt, Scalar(0, 255, 255), 3);

    std::ostringstream ss;
    ss << std::setprecision(3) << distance;
    putText(src, "Distance: " + ss.str() + " m",
      Point(20, height - 60), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 2);
    putText(src, "Offset X: " + std::to_string(offset_x) + " Y: " + std::to_string(offset_y),
      Point(20, height - 20), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 2);

    auto pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.pose           = pole_pose.pose;
    pose_stamped.header.stamp   = this->get_clock()->now();
    pose_stamped.header.frame_id = "map";
    pole_pose_only_publisher_->publish(pose_stamped);
  }
}

void SlalomDetector::debugPublish(cv::Mat & src, image_transport::Publisher & publisher)
{
  sensor_msgs::msg::Image debug_msg;
  cv_bridge::CvImage debug_image;
  cv::Size sz = src.size();
  debug_image.image = src;
  debug_image.toImageMsg(debug_msg);
  debug_msg.width    = sz.width;
  debug_msg.height   = sz.height;
  debug_msg.encoding = src.channels() == 1
    ? sensor_msgs::image_encodings::MONO8
    : sensor_msgs::image_encodings::BGR8;
  debug_msg.step              = sz.width * src.channels();
  debug_msg.header.frame_id   = "base_link";
  publisher.publish(debug_msg);
}

}  // namespace steelhead_slaloms
#ifndef TRITON_CONTROL__STRAIGHT_LINE_NAVIGATOR
#define TRITON_CONTROL__STRAIGHT_LINE_NAVIGATOR

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "triton_interfaces/msg/waypoint.hpp"
#include "triton_interfaces/msg/trajectory_type.hpp"
#include <math.h>
#include <chrono>

#define STRAIGHT_CALIBRATING 0
#define STRAIGHT_COOLDOWN 1
#define STRAIGHT_NAVIGATING 2
#define STRAIGHT_COMPLETED 3

namespace triton_controls
{
    class StraightLineNavigator : public rclcpp::Node
    {
    public:
        explicit StraightLineNavigator(const rclcpp::NodeOptions & options);

    private:
        void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void navigation_timer_callback();

        void calibrate_compass();
        void cooldown_wait();
        void navigate_straight();
        double get_yaw_from_pose(const geometry_msgs::msg::Pose& pose);

        rclcpp::Publisher<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_publisher_;
        rclcpp::Publisher<triton_interfaces::msg::TrajectoryType>::SharedPtr mode_publisher_;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
        
        rclcpp::TimerBase::SharedPtr navigation_timer_;

        uint8_t navigation_state_;
        geometry_msgs::msg::Pose current_pose_;
        bool pose_initialized_;
        
        // Compass calibration
        double calibrated_heading_;
        bool compass_calibrated_;
        
        // Timing
        std::chrono::steady_clock::time_point state_start_time_;
        std::chrono::milliseconds cooldown_duration_;
        
        // Navigation parameters
        double navigation_distance_;
        double forward_speed_;
    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::StraightLineNavigator)

#endif  //TRITON_CONTROL__STRAIGHT_LINE_NAVIGATOR
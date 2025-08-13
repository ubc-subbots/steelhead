#ifndef TRITON_CONTROL__SIMPLE_GATE_NAVIGATOR
#define TRITON_CONTROL__SIMPLE_GATE_NAVIGATOR

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "triton_interfaces/msg/waypoint.hpp"
#include "triton_interfaces/msg/object_offset.hpp"
#include "triton_interfaces/msg/trajectory_type.hpp"
#include <math.h>
#include <chrono>

#define GATE_SEARCHING 0
#define GATE_APPROACHING 1
#define GATE_THRUSTING 2
#define GATE_COMPLETED 3

namespace triton_controls
{
    class SimpleGateNavigator : public rclcpp::Node
    {
    public:
        explicit SimpleGateNavigator(const rclcpp::NodeOptions & options);

    private:
        void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void gate_callback(const triton_interfaces::msg::ObjectOffset::SharedPtr msg);
        void navigation_timer_callback();

        void search_for_gate();
        void approach_gate();
        void navigate_through_gate();
        double calculate_distance_to_gate();
        double get_yaw_from_pose(const geometry_msgs::msg::Pose& pose);

        rclcpp::Publisher<triton_interfaces::msg::Waypoint>::SharedPtr waypoint_publisher_;
        rclcpp::Publisher<triton_interfaces::msg::TrajectoryType>::SharedPtr mode_publisher_;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
        rclcpp::Subscription<triton_interfaces::msg::ObjectOffset>::SharedPtr gate_subscription_;
        
        rclcpp::TimerBase::SharedPtr navigation_timer_;

        uint8_t gate_state_;
        geometry_msgs::msg::Pose current_pose_;
        geometry_msgs::msg::Pose gate_pose_;
        bool gate_detected_;
        bool pose_initialized_;
        
        double navigation_distance_;
        
        // Gate loss detection and straight thrust
        std::chrono::steady_clock::time_point last_gate_seen_time_;
        std::chrono::steady_clock::time_point thrust_start_time_;
        std::chrono::milliseconds gate_loss_timeout_;
        std::chrono::milliseconds thrust_duration_;
        bool gate_ever_detected_;
    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::SimpleGateNavigator)

#endif  //TRITON_CONTROL__SIMPLE_GATE_NAVIGATOR
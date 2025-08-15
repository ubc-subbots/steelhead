#ifndef TRITON_CONTROL__HOVER_UNDERWATER
#define TRITON_CONTROL__HOVER_UNDERWATER

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "triton_interfaces/msg/waypoint.hpp"
#include "triton_interfaces/msg/object_offset.hpp"
#include "triton_interfaces/msg/trajectory_type.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <math.h>

namespace triton_controls
{

    class HoverUnderwater : public rclcpp::Node
    {

    public:

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_;
        /** Constructor
         * 
         * Creates the hover underwater controller from the given parameters, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit HoverUnderwater(const rclcpp::NodeOptions & options);

    private:

         /** State message callback
         * 
         * Updates private variable containing the AUV's current pose. 
         * 
         * @param msg sensor_msgs imu message with orientation
         */
        void state_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr state_subscription_;
        
        // Control state variables
        tf2::Quaternion initial_orientation_;
        bool set_;
        bool started_;
        bool stopped_;
        bool initial_orientation_set_ = false;
        rclcpp::Time start_time_;
        rclcpp::Time control_start_time_;
        
        // Control parameters
        double delay_seconds_;
        double dive_seconds_;
        double hover_seconds_;
        double surface_seconds_;
        double kp_roll_;
        double kp_pitch_;
        double kp_yaw_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::HoverUnderwater)

#endif  //TRITON_CONTROL__HOVER_UNDERWATER
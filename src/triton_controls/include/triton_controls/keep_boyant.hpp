#ifndef TRITON_CONTROL__KEEP_BUOYANT
#define TRITON_CONTROL__KEEP_BUOYANT

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

    class KeepBoyant : public rclcpp::Node
    {

    public:

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_;
        /** Constructor
         * 
         * Creates the trajectory generator from the given parameters, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit KeepBoyant(const rclcpp::NodeOptions & options);

    private:

         /** State message callback
         * 
         * Updates private variable containing the AUV's current pose. 
         * 
         * @param msg geometry pose message with position and orientation
         */
        void state_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr state_subscription_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::KeepBoyant)

#endif  //TRITON_CONTROL__TRAJECTORY_GENERATOR

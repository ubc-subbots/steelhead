#ifndef TRITON_CONTROL__WAYPOINT_MARKER
#define TRITON_CONTROL__WAYPOINT_MARKER

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "triton_interfaces/msg/pipeline_feedback.hpp"

namespace triton_controls
{

    class WaypointMarker : public rclcpp::Node
    {

    public:

        /** Constructor
         * 
         * Creates the waypoint from the given parameters, then sets up pubs/subs
         * 
         * @param options ros2 node options.
         */
        explicit WaypointMarker(const rclcpp::NodeOptions & options);

    private:

         /** Pose message callback
         * 
         * Publishes the waypoint position calculated by taking the waypoint
         * and adding the initial position given in the first message recieved.
         * 
         * @param msg geometry pose message witth position and orientation
         */
        void callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> & parameters);

        rclcpp::Publisher<triton_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

        const int max_waypoints_ = 20; //later lets try other data structure like hash table?
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        bool has_pose_;
        geometry_msgs::msg::Pose pose_;

        int num_waypoints_;
        std::vector<std::vector<double>> waypoint_values_;

        std::string threshold_type_;
        double threshold_dist_;
        double threshold_ang_diff_;
        int threshold_consect_;

        int threshold_counter_;

    };

} // namespace triton_controls

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(triton_controls::WaypointMarker)

#endif  //TRITON_CONTROL__WAYPOINT_MARKER
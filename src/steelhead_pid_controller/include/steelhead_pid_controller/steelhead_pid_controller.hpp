#ifndef STEELHEAD_PID_CONTROLLER__
#define STEELHEAD_PID_CONTROLLER__
#pragma once

#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace steelhead_pid_controller
{

class PidController : public rclcpp::Node
{
public:
    explicit PidController(const rclcpp::NodeOptions & options);

    ~PidController();

private:

    void control_loop();
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_;
    geometry_msgs::msg::Pose::SharedPtr cur_pose;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    // steady_clock: monotonic, immune to NTP/system clock jumps corrupting dt
    std::chrono::time_point<std::chrono::steady_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    void pose_update(const geometry_msgs::msg::Pose::SharedPtr msg);
    struct PID
    {
        void load(float p, float i, float d, float imax = 0.0f)
        {
            Kp = p;
            Ki = i;
            Kd = d;
            i_max = imax;
        }

        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float sum_error = 0;
        float last_error = 0;
        float i_max = 0;  // integral clamp magnitude; <= 0 disables clamping
        bool angular = false;  // error is an angle in [-pi, pi]; wrap the derivative across +/-pi
        float update(float error, float dt)
        {
            if (!std::isfinite(error) || dt <= 0.0f) return 0.0f;

            float diff_error = error - last_error;
            if (angular) {
                if (diff_error > M_PI) diff_error -= 2.0f * M_PI;
                else if (diff_error < -M_PI) diff_error += 2.0f * M_PI;
            }
            sum_error += error * dt;
            if (i_max > 0.0f) sum_error = std::max(-i_max, std::min(sum_error, i_max));
            float ret = Kp * error + Ki * sum_error + Kd * diff_error / dt;
            last_error = error;
            return std::isfinite(ret) ? ret : 0.0f;
        }
    };

    PID pid_force_x;
    PID pid_force_y;
    PID pid_force_z;
    PID pid_roll;
    PID pid_pitch;
    PID pid_yaw;
};
}  // namespace steelhead_pid_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_pid_controller::PidController)

#endif

#include <chrono>
#include <cmath>
#include <memory>
#include <steelhead_pid_controller/steelhead_pid_controller.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace steelhead_pid_controller
{

  PidController::PidController(const rclcpp::NodeOptions & options) 
  : Node("pid_controller", options)
  {
    last_time_ = std::chrono::high_resolution_clock::now();
    auto control_loop_time = 5ms;
    control_loop_timer_ = create_wall_timer(control_loop_time, 
      std::bind(&PidController::control_loop, this));

    // ROS2 setup
    sub_ = create_subscription<geometry_msgs::msg::Pose>(
        "/steelhead/controls/input_pose",
        10,
        std::bind(&PidController::pose_update, this, _1));

    pub_ = create_publisher<geometry_msgs::msg::Wrench>(
        "/steelhead/controls/input_forces",
        10);

    RCLCPP_INFO(this->get_logger(), "PID Controller starting!");

    float x_p, x_i, x_d;
    float y_p, y_i, y_d;
    float z_p, z_i, z_d;
    float roll_p, roll_i, roll_d;
    float pitch_p, pitch_i, pitch_d;
    float yaw_p, yaw_i, yaw_d;
    this->declare_parameter("force_x_p", x_p);
    this->declare_parameter("force_x_i", x_i);
    this->declare_parameter("force_x_d", x_d);
    this->declare_parameter("force_y_p", y_p);
    this->declare_parameter("force_y_i", y_i);
    this->declare_parameter("force_y_d", y_d);
    this->declare_parameter("force_z_p", z_p);
    this->declare_parameter("force_z_i", z_i);
    this->declare_parameter("force_z_d", z_d);
    this->declare_parameter("force_roll_p", roll_p);
    this->declare_parameter("force_roll_i", roll_i);
    this->declare_parameter("force_roll_d", roll_d);
    this->declare_parameter("force_pitch_p", pitch_p);
    this->declare_parameter("force_pitch_i", pitch_i);
    this->declare_parameter("force_pitch_d", pitch_d);
    this->declare_parameter("force_yaw_p", yaw_p);
    this->declare_parameter("force_yaw_i", yaw_i);
    this->declare_parameter("force_yaw_d", yaw_d);

    this->get_parameter("force_x_p", x_p);
    this->get_parameter("force_x_i", x_i);
    this->get_parameter("force_x_d", x_d);
    this->get_parameter("force_y_p", y_p);
    this->get_parameter("force_y_i", y_i);
    this->get_parameter("force_y_d", y_d);
    this->get_parameter("force_z_p", z_p);
    this->get_parameter("force_z_i", z_i);
    this->get_parameter("force_z_d", z_d);
    this->get_parameter("force_roll_p", roll_p);
    this->get_parameter("force_roll_i", roll_i);
    this->get_parameter("force_roll_d", roll_d);
    this->get_parameter("force_pitch_p", pitch_p);
    this->get_parameter("force_pitch_i", pitch_i);
    this->get_parameter("force_pitch_d", pitch_d);
    this->get_parameter("force_yaw_p", yaw_p);
    this->get_parameter("force_yaw_i", yaw_i);
    this->get_parameter("force_yaw_d", yaw_d);

    pid_force_x.load(x_p, x_i, x_d);
    pid_force_y.load(y_p, y_i, y_d);
    pid_force_z.load(z_p, z_i, z_d);
    pid_roll.load(roll_p, roll_i, roll_d);
    pid_pitch.load(pitch_p, pitch_i, pitch_d);
    pid_yaw.load(yaw_p, yaw_i, yaw_d);

    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PidController::parametersCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PID Controller successfully started!");
  }

  PidController::~PidController()
  {
  }

  void PidController::pose_update(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    cur_pose = msg;
  }

  void PidController::control_loop()
  {
    if (!cur_pose)
    {
      return;
    }

    auto now = std::chrono::high_resolution_clock::now();
    float dt = 
      std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count() / 1e6;

    tf2::Quaternion current_pose_q(
      cur_pose->orientation.x,
      cur_pose->orientation.y,
      cur_pose->orientation.z,
      cur_pose->orientation.w);
    tf2::Matrix3x3 current_pose_q_m(current_pose_q);
    double current_pose_roll, current_pose_pitch, current_pose_yaw;
    current_pose_q_m.getRPY(current_pose_roll, current_pose_pitch, current_pose_yaw);

    float cur_yaw = 0;
    if (!std::isnan(current_pose_yaw)) 
    {
      cur_yaw = -current_pose_yaw; // TODO: why negative
    }

    float pos_x_error = cur_pose->position.x;
    float pos_y_error = cur_pose->position.y;
    float pos_z_error = cur_pose->position.z;
    float roll_error = !std::isnan(current_pose_roll) ? current_pose_roll : 0;
    float pitch_error = !std::isnan(current_pose_pitch) ? current_pose_pitch : 0;
    float yaw_error = cur_yaw;

    float forceX = pid_force_x.update(pos_x_error, dt);
    float forceY = pid_force_y.update(pos_y_error, dt);
    float forceZ = pid_force_z.update(pos_z_error, dt);
    float torqueX = pid_roll.update(roll_error, dt);
    float torqueY = pid_roll.update(pitch_error, dt);
    float torqueZ = pid_yaw.update(yaw_error, dt);

    last_time_ = std::chrono::high_resolution_clock::now();

    geometry_msgs::msg::Wrench wrenchOut;
    wrenchOut.force.x = forceX;
    wrenchOut.force.y = forceY;
    wrenchOut.force.z = forceZ;
    wrenchOut.torque.x = torqueX;
    wrenchOut.torque.y = torqueY;
    wrenchOut.torque.z = torqueZ;
    pub_->publish(wrenchOut);
  }

  rcl_interfaces::msg::SetParametersResult PidController::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
      if (param.get_name() == "force_x_p") pid_force_x.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_x_i") pid_force_x.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_x_d") pid_force_x.Kd = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_y_p") pid_force_y.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_y_i") pid_force_y.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_y_d") pid_force_y.Kd = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_z_p") pid_force_z.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_z_i") pid_force_z.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_z_d") pid_force_z.Kd = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_roll_p") pid_roll.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_roll_i") pid_roll.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_roll_d") pid_roll.Kd = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_pitch_p") pid_pitch.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_pitch_i") pid_pitch.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_pitch_d") pid_pitch.Kd = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_yaw_p") pid_yaw.Kp = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_yaw_i") pid_yaw.Ki = static_cast<float>(param.as_double());
      else if (param.get_name() == "force_yaw_d") pid_yaw.Kd = static_cast<float>(param.as_double());
    }

    return result;
  }
}  // namespace steelhead_pid_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<steelhead_pid_controller::PidController>(options));
  rclcpp::shutdown();
  return 0;
}

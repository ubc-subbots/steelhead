#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
class ClawPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        model_ = _model;
        
        // Initialize ROS2 node
        ros_node_ = gazebo_ros::Node::Get(_sdf);
        
        // Get joint from SDF
        std::string joint_name = "claw_joint";
        if (_sdf->HasElement("joint_name"))
            joint_name = _sdf->GetElement("joint_name")->Get<std::string>();
            
        joint_ = model_->GetJoint(joint_name);
        if (!joint_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] not found!", joint_name.c_str());
            return;
        }
        
        // Set up ROS2 communication
        command_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
            "/steelhead/claw/position_command", 10,
            std::bind(&ClawPlugin::on_command, this, std::placeholders::_1));
        
        position_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>(
            "/steelhead/claw/position", 10);
        
        // Connect to Gazebo update event
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ClawPlugin::on_update, this));
            
        RCLCPP_INFO(ros_node_->get_logger(), "Claw plugin loaded for joint: %s", joint_name.c_str());
    }

private:
    void on_command(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_position_ = std::max(0.0, std::min(1.0, msg->data));
    }
    
    void on_update()
    {
        if (!joint_) return;
        
        // Get current position
        double current_angle = joint_->Position(0);
        
        // Convert 0-1 range to actual joint angles
        // Adjust these values based on your claw's joint limits
        double min_angle = 0.0;     // Closed position (radians)
        double max_angle = 1.57;    // Open position (90 degrees)
        double target_angle = min_angle + target_position_ * (max_angle - min_angle);
        
        // Simple PD control
        double error = target_angle - current_angle;
        double force = kp_ * error + kd_ * (error - prev_error_) / 0.001; // Assuming 1ms timestep
        prev_error_ = error;
        
        joint_->SetForce(0, force);
        
        // Publish current position (normalized to 0-1)
        auto pos_msg = std_msgs::msg::Float64();
        pos_msg.data = (current_angle - min_angle) / (max_angle - min_angle);
        position_pub_->publish(pos_msg);
    }

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    gazebo_ros::Node::SharedPtr ros_node_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    
    event::ConnectionPtr update_connection_;
    
    double target_position_ = 0.0;
    double prev_error_ = 0.0;
    double kp_ = 1000.0;  // Proportional gain
    double kd_ = 100.0;   // Derivative gain
};

GZ_REGISTER_MODEL_PLUGIN(ClawPlugin)
}
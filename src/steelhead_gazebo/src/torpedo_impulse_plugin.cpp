#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <thread>
#include <string>

namespace steelhead_gazebo
{
class TorpedoImpulsePlugin : public gazebo::ModelPlugin
{
public:
  TorpedoImpulsePlugin() = default;

  ~TorpedoImpulsePlugin() override {}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model = _model;

    // ---- Read params ----
    if (_sdf->HasElement("link_name"))
      link_name = _sdf->Get<std::string>("link_name");
    else
    {
      gzerr << "link_name not specified for torpedo impulse plugin.\n";
      return;
    }

    force_n = _sdf->HasElement("force_n") ? _sdf->Get<double>("force_n") : 40.0;
    duration_s = _sdf->HasElement("duration_s") ? _sdf->Get<double>("duration_s") : 0.05;
    fire_on_spawn = _sdf->HasElement("fire_on_spawn") ? _sdf->Get<bool>("fire_on_spawn") : false;

    // direction_body is optional; default +X
    direction_body = ignition::math::Vector3d(1, 0, 0);
    if (_sdf->HasElement("direction_body"))
      direction_body = _sdf->Get<ignition::math::Vector3d>("direction_body");

    // ---- ROS namespace/topic (same pattern as ThrusterDriver) ----
    std::string ns = "/steelhead/torpedo";
    std::string topic = "fire";
    if (_sdf->HasElement("ros"))
    {
      sdf::ElementPtr ros_sdf = _sdf->GetElement("ros");
      if (ros_sdf->HasElement("namespace")) ns = ros_sdf->Get<std::string>("namespace");
      if (ros_sdf->HasElement("trigger_topic")) topic = ros_sdf->Get<std::string>("trigger_topic");
    }
    topic_name = ns + "/" + topic;

    // ---- Get link ----
    link = model->GetLink(link_name);
    if (!link)
    {
      gzerr << "Could not find link: " << link_name << "\n";
      return;
    }

    node = rclcpp::Node::make_shared("torpedo_impulse_" + model->GetName());

    // ---- Subscribe ----
    sub = node->create_subscription<std_msgs::msg::Empty>(
      topic_name, 10,
      std::bind(&TorpedoImpulsePlugin::OnFire, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(), "Listening on %s", topic_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Applying %.2f N for %.3f s", force_n, duration_s);

    // ---- Hook update loop ----
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TorpedoImpulsePlugin::ApplyForce, this));

    if (fire_on_spawn)
    {
      firing = true;
      end_time = model->GetWorld()->SimTime() + gazebo::common::Time(duration_s);
    }

    // ---- Spin ROS in background ----
    spinThread = std::thread(std::bind(&TorpedoImpulsePlugin::SpinNode, this));
  }

private:
  void OnFire(const std_msgs::msg::Empty::SharedPtr)
  {
    firing = true;
    end_time = model->GetWorld()->SimTime() + gazebo::common::Time(duration_s);
  }

  void ApplyForce()
  {
    if (!firing) return;

    auto now = model->GetWorld()->SimTime();
    if (now > end_time)
    {
      firing = false;
      return;
    }

    // body direction -> world direction
    ignition::math::Vector3d dir_world =
      link->WorldPose().Rot().RotateVector(direction_body);

    // Apply as a normal force (same style as thrusters)
    link->AddLinkForce(dir_world * force_n);
  }

  void SpinNode()
  {
    rclcpp::spin(node);
  }

private:
  gazebo::physics::ModelPtr model;
  gazebo::physics::LinkPtr link;
  gazebo::event::ConnectionPtr updateConnection;

  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub;
  std::thread spinThread;

  std::string link_name;
  std::string topic_name;

  double force_n{40.0};
  double duration_s{0.05};
  ignition::math::Vector3d direction_body{1, 0, 0};
  bool fire_on_spawn{false};

  bool firing{false};
  gazebo::common::Time end_time;
};

GZ_REGISTER_MODEL_PLUGIN(TorpedoImpulsePlugin)
}  // namespace steelhead_gazebo

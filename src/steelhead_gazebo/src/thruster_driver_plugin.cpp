#include "steelhead_gazebo/thruster_driver_plugin.hpp"
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>

GZ_ADD_PLUGIN(steelhead_gazebo::ThrusterDriver,
              gz::sim::System,
              steelhead_gazebo::ThrusterDriver::ISystemConfigure,
              steelhead_gazebo::ThrusterDriver::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(steelhead_gazebo::ThrusterDriver, "steelhead_gazebo::ThrusterDriver")

namespace steelhead_gazebo
{

    ThrusterDriver::ThrusterDriver() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node = rclcpp::Node::make_shared("thruster_driver");
    }

    ThrusterDriver::~ThrusterDriver() {
        if (this->node) rclcpp::shutdown();
        if (this->spinThread.joinable()) this->spinThread.join();
    }

    void ThrusterDriver::Configure(const gz::sim::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::EventManager &/*_eventMgr*/)
    {
        if (_sdf->HasElement("thruster_count"))
        {
            this->thruster_count = _sdf->Get<unsigned int>("thruster_count");
        }
        else
        {
            gzerr << "thruster_count value not specified, exiting.\n";
            exit(1);
        }

        std::shared_ptr<const sdf::Element> ros_namespace = _sdf->FindElement("ros");
        this->GetRosNamespace(ros_namespace);

        this->thrust_values = std::vector<double>(this->thruster_count, 0);
        this->force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                            this->topic_name, 
                            10, 
                            std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Listening on %s\n", this->topic_name.c_str());

        gz::sim::Model model(_entity);
        std::string model_name = model.Name(_ecm);

        for (unsigned int i = 1; i <= thruster_count; i++)
        {
            std::string nested_model_name = "thruster" + std::to_string(i);
            gz::sim::Entity nested_model_entity = model.ModelByName(_ecm, nested_model_name);
            
            if (nested_model_entity == gz::sim::kNullEntity) {
                gzerr << "Failed to find nested model: " << nested_model_name << std::endl;
                continue;
            }
            
            gz::sim::Model nested_model(nested_model_entity);
            gz::sim::Entity link_entity = nested_model.LinkByName(_ecm, "thruster");
            
            if (link_entity != gz::sim::kNullEntity) {
                this->thruster.push_back(link_entity);
                _ecm.CreateComponent(link_entity, gz::sim::components::WorldPose());
                RCLCPP_INFO(node->get_logger(), "Found link for %s", nested_model_name.c_str());
            } else {
                gzerr << "Failed to find link 'thruster' inside nested model: " << nested_model_name << std::endl;
            }
        }
    
        this->spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }

    void ThrusterDriver::GetRosNamespace(std::shared_ptr<const sdf::Element> ros_sdf)
    {
        std::string _namespace;
        std::string topic;
    
        if (ros_sdf && ros_sdf->HasElement("namespace"))
        {
            _namespace = ros_sdf->Get<std::string>("namespace");
        }
        else
        {
            _namespace = "steelhead/steelhead_gazebo";
        }

        if (ros_sdf && ros_sdf->HasElement("remapping"))
        {
            topic = ros_sdf->Get<std::string>("remapping");
        }
        else
        {
            topic = "thruster_values";
        }

        this->topic_name = _namespace + "/" + topic;
    }

    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::ConstSharedPtr joint_cmd)
    {
        if (joint_cmd->data.size() != this->thruster_count)
        {
            RCLCPP_WARN(node->get_logger(), "message size does not match thruster count, ignoring command.\n");
            return;
        }

        for (unsigned int i = 0; i < this->thruster_count; i++)
        {
            this->thrust_values[i] = joint_cmd->data[i];
        }
    }

    void ThrusterDriver::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        for (unsigned int i = 0; i < this->thruster.size(); i++)
        {
            gz::sim::Link link(this->thruster[i]);
            auto pose = link.WorldPose(_ecm);
            if (pose)
            {
                gz::math::Vector3d localForce(0, 0, this->thrust_values[i]);
                gz::math::Vector3d worldForce = pose->Rot() * localForce;
                link.AddWorldForce(_ecm, worldForce);
            }
        }
    }

    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

}

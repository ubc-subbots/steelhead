#include "steelhead_gazebo/thruster_driver_plugin.hpp"

namespace steelhead_gazebo
{

    ThrusterDriver::ThrusterDriver() : node{rclcpp::Node::make_shared("thruster_driver")} {}


    ThrusterDriver::~ThrusterDriver() {}


    void ThrusterDriver::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

        sdf::ElementPtr ros_namespace = _sdf->GetElement("ros");
        this->GetRosNamespace(ros_namespace);

        this->thrust_values = std::vector<double>(this->thruster_count, 0);
        this->force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                            this->topic_name, 
                            10, 
                            std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Listening on " + this->topic_name + "\n");

        std::string model_name = _model->GetName();

        for (unsigned int i = 1; i <= thruster_count; i++)
        {
            std::string thruster_name = model_name + "::thruster" + std::to_string(i) + "::thruster";
            this->thruster.push_back(_model->GetLink(thruster_name));
        }

        // Fixed thruster poses relative to the base link, one <thruster_pose>
        // per thruster in order. Used for thrusters without a dedicated link,
        // so the model can be a single rigid body (no fixed joints for the
        // constraint solver to snap).
        if (_sdf->HasElement("thruster_pose"))
        {
            sdf::ElementPtr pose_elem = _sdf->GetElement("thruster_pose");
            while (pose_elem != nullptr)
            {
                this->thruster_offset.push_back(pose_elem->Get<ignition::math::Pose3d>());
                pose_elem = pose_elem->GetNextElement("thruster_pose");
            }
        }

        // Resolve the main body link the same way the hydrodynamics plugin
        // does, via the model-level <base_link> element, falling back to the
        // heaviest link in the model. Never exit: without a base link the
        // plugin still runs by loading the thruster links directly.
        sdf::ElementPtr model_sdf = _model->GetSDF();
        if (model_sdf->HasElement("base_link"))
        {
            this->base_link = _model->GetLink(model_sdf->Get<std::string>("base_link"));
        }
        if (this->base_link == nullptr)
        {
            for (auto const & link : _model->GetLinks())
            {
                if (this->base_link == nullptr ||
                    link->GetInertial()->Mass() > this->base_link->GetInertial()->Mass())
                {
                    this->base_link = link;
                }
            }
        }
        if (this->base_link == nullptr)
        {
            gzwarn << "ThrusterDriver: no base link found, applying thrust to "
                      "the thruster links directly (can destabilize physics).\n";
        }
        else
        {
            unsigned int via_link = 0, via_pose = 0, unresolved = 0;
            for (unsigned int i = 0; i < thruster_count; i++)
            {
                if (this->thruster[i] != nullptr) via_link++;
                else if (i < this->thruster_offset.size()) via_pose++;
                else unresolved++;
            }
            gzmsg << "ThrusterDriver: applying thrust to link '"
                  << this->base_link->GetName() << "' (" << via_link
                  << " thrusters via links, " << via_pose << " via fixed poses).\n";
            if (unresolved > 0)
            {
                gzerr << "ThrusterDriver: " << unresolved << " thrusters have "
                         "neither a link nor a <thruster_pose>; they will produce no thrust.\n";
            }
        }

        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&ThrusterDriver::ApplyForce, this));

        /// @todo feels like there should be a way to pass rclcpp::spin directly to thread, this works the way it is though 
        this->spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }


    void ThrusterDriver::GetRosNamespace(sdf::ElementPtr ros_sdf)
    {
        std::string _namespace;
        std::string topic;
    
        if (ros_sdf->HasElement("namespace"))
        {
            _namespace = ros_sdf->Get<std::string>("namespace");
        }
        else
        {
            _namespace = "steelhead/steelhead_gazebo";
        }

        if (ros_sdf->HasElement("remapping"))
        {
            topic = ros_sdf->Get<std::string>("remapping");
        }
        else
        {
            topic = "thruster_values";
        }

        this->topic_name = _namespace + "/" + topic;
    }


    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd)
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


    void ThrusterDriver::ApplyForce()
    {
        for (unsigned int i = 0; i < thruster_count; i++)
        {
            // Thrust acts along the thruster's local z axis. The pose comes
            // from the thruster link when the model has one, otherwise from
            // the configured fixed offset relative to the base link.
            ignition::math::Pose3d pose;
            if (this->thruster[i] != nullptr)
            {
                pose = this->thruster[i]->WorldCoGPose();
            }
            else if (this->base_link != nullptr && i < this->thruster_offset.size())
            {
                pose = this->thruster_offset[i] + this->base_link->WorldPose();
            }
            else
            {
                continue;
            }

            if (this->base_link != nullptr)
            {
                // Apply the force to the heavy base link at the thruster's
                // location: the net wrench on the vehicle is identical to
                // loading a thruster link itself, but near-massless thruster
                // links never carry the force (loading them directly violates
                // the fixed-joint constraints and the model explodes).
                ignition::math::Vector3d world_force =
                    pose.Rot().RotateVector(ignition::math::Vector3d(0, 0, this->thrust_values[i]));
                this->base_link->AddForceAtWorldPosition(world_force, pose.Pos());
            }
            else
            {
                this->thruster[i]->AddLinkForce(ignition::math::Vector3d(0, 0, this->thrust_values[i]));
            }
        }
    }


    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

}

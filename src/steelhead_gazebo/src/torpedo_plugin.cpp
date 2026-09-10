#include "steelhead_gazebo/torpedo_plugin.hpp"

namespace steelhead_gazebo
{

    TorpedoPlugin::TorpedoPlugin() {}


    TorpedoPlugin::~TorpedoPlugin() {}


    void TorpedoPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model_ = _model;
        this->world_ = this->model_->GetWorld();
        
        std::string node_name = "torpedo_plugin_" + this->model_->GetName();
        this->ros_node_ = rclcpp::Node::make_shared(node_name);

        std::string link_name = "base_link";
        if (_sdf->HasElement("link_name"))
        {
            link_name = _sdf->Get<std::string>("link_name");
        }
        
        this->link_ = this->model_->GetLink(link_name);
        if (!this->link_)
        {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Link '%s' not found!\n", link_name.c_str());
            return;
        }

        this->initial_force_ = 500.0;
        if (_sdf->HasElement("initial_force"))
        {
            this->initial_force_ = _sdf->Get<double>("initial_force");
        }

        this->force_duration_ = 0.5;
        if (_sdf->HasElement("force_duration"))
        {
            this->force_duration_ = _sdf->Get<double>("force_duration");
        }

        this->lifetime_ = 5.0;
        if (_sdf->HasElement("lifetime"))
        {
            this->lifetime_ = _sdf->Get<double>("lifetime");
        }
        
        this->spawn_time_ = this->world_->SimTime();
        
        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&TorpedoPlugin::OnUpdate, this));
        
        // RCLCPP_INFO(this->ros_node_->get_logger(), "Torpedo plugin loaded. Force: %.2f, Lifetime: %.2f\n", this->initial_force_, this->lifetime_);
    }


    void TorpedoPlugin::OnUpdate()
    {
        gazebo::common::Time current_time = this->world_->SimTime();
        double elapsed = (current_time - this->spawn_time_).Double();

        if (elapsed < this->force_duration_)
        {
            this->link_->AddRelativeForce(ignition::math::Vector3d(this->initial_force_, 0.0, 0.0));
        }

        if (elapsed >= this->lifetime_)
        {
            this->world_->RemoveModel(this->model_->GetName());
        }
    }

}
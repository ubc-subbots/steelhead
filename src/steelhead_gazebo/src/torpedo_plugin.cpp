#include "steelhead_gazebo/torpedo_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>

GZ_ADD_PLUGIN(
    steelhead_gazebo::TorpedoPlugin,
    gz::sim::System,
    steelhead_gazebo::TorpedoPlugin::ISystemConfigure,
    steelhead_gazebo::TorpedoPlugin::ISystemPreUpdate)

namespace steelhead_gazebo
{

    TorpedoPlugin::TorpedoPlugin() {}

    TorpedoPlugin::~TorpedoPlugin() {}

    void TorpedoPlugin::Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr)
    {
        this->model_ = gz::sim::Model(_entity);
        if (!this->model_.Valid(_ecm))
        {
            return;
        }

        std::string node_name = "torpedo_plugin_" + this->model_.Name(_ecm);
        this->ros_node_ = rclcpp::Node::make_shared(node_name);

        std::string link_name = "base_link";
        if (_sdf->HasElement("link_name"))
        {
            link_name = _sdf->Get<std::string>("link_name");
        }
        
        this->link_entity_ = this->model_.LinkByName(_ecm, link_name);
        if (this->link_entity_ == gz::sim::kNullEntity)
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
        
        // spawn time will be set on first update
        this->spawn_time_ = std::chrono::steady_clock::duration::zero();
    }

    void TorpedoPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        if (this->spawn_time_ == std::chrono::steady_clock::duration::zero())
        {
            this->spawn_time_ = _info.simTime;
        }

        double elapsed = std::chrono::duration<double>(_info.simTime - this->spawn_time_).count();

        if (elapsed < this->force_duration_)
        {
            // Get pose of the link to apply force in local frame
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->link_entity_);
            gz::math::Pose3d pose = poseComp ? poseComp->Data() : gz::math::Pose3d::Zero;
            
            gz::math::Vector3d force_local(this->initial_force_, 0.0, 0.0);
            gz::math::Vector3d force_world = pose.Rot() * force_local;

            gz::msgs::Wrench wrench;
            gz::msgs::Set(wrench.mutable_force(), force_world);
            gz::msgs::Set(wrench.mutable_torque(), gz::math::Vector3d::Zero);
            _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(this->link_entity_, wrench);
        }

        if (elapsed >= this->lifetime_)
        {
            // In gz-sim, deleting an entity requires requesting it via event manager or ECM
            _ecm.RequestRemoveEntity(this->model_.Entity());
        }
    }

}

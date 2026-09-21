#include "steelhead_gazebo/torpedo_plugin.hpp"
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

GZ_ADD_PLUGIN(
    steelhead_gazebo::TorpedoPlugin,
    gz::sim::System,
    steelhead_gazebo::TorpedoPlugin::ISystemConfigure,
    steelhead_gazebo::TorpedoPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(steelhead_gazebo::TorpedoPlugin, "steelhead_gazebo::TorpedoPlugin")

namespace steelhead_gazebo
{
    void TorpedoPlugin::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &/*_eventMgr*/)
    {
        this->model_ = gz::sim::Model(_entity);
        
        std::string link_name = "torpedo";
        if (_sdf->HasElement("link_name"))
        {
            link_name = _sdf->Get<std::string>("link_name");
        }
        
        gz::sim::Entity linkEntity = this->model_.LinkByName(_ecm, link_name);
        if (linkEntity != gz::sim::kNullEntity) {
            this->link_ = gz::sim::Link(linkEntity);
            this->link_.EnableVelocityChecks(_ecm, true);
        }

        if (_sdf->HasElement("initial_force"))
        {
            this->initial_force_ = _sdf->Get<double>("initial_force");
        }
        if (_sdf->HasElement("force_duration"))
        {
            this->force_duration_ = _sdf->Get<double>("force_duration");
        }
        if (_sdf->HasElement("lifetime"))
        {
            this->lifetime_ = _sdf->Get<double>("lifetime");
        }
        
        // Ensure WorldPose component is created
        _ecm.CreateComponent(this->model_.Entity(), gz::sim::components::Pose());
    }

    void TorpedoPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        if (this->spawn_time_.count() == 0) {
            this->spawn_time_ = _info.simTime;
        }

        std::chrono::duration<double> elapsed = _info.simTime - this->spawn_time_;

        if (elapsed.count() < this->force_duration_)
        {
            if (this->link_.Entity() != gz::sim::kNullEntity) {
                auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model_.Entity());
                if (poseComp) {
                    gz::math::Vector3d force_local(this->initial_force_, 0.0, 0.0);
                    gz::math::Vector3d force_world = poseComp->Data().Rot() * force_local;
                    this->link_.AddWorldWrench(_ecm, force_world, gz::math::Vector3d::Zero);
                }
            }
        }

        if (elapsed.count() >= this->lifetime_)
        {
            _ecm.RequestRemoveEntity(this->model_.Entity());
        }
    }
}

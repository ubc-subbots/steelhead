#ifndef STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP
#define STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

namespace steelhead_gazebo
{
    class TorpedoPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate
    {
    public:
        TorpedoPlugin() = default;
        ~TorpedoPlugin() override = default;

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    private:
        gz::sim::Model model_{gz::sim::kNullEntity};
        gz::sim::Link link_{gz::sim::kNullEntity};
        std::chrono::steady_clock::duration spawn_time_{0};
        double initial_force_{50.0};
        double force_duration_{0.05};
        double lifetime_{15.0};
    };
}
#endif

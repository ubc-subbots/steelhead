#ifndef STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP
#define STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP

#include <string>
#include <chrono>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include "rclcpp/rclcpp.hpp"

namespace steelhead_gazebo
{

    class TorpedoPlugin : 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {

    public:

        // Constructor
        TorpedoPlugin();

        // Destructor
        ~TorpedoPlugin() override;

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    private:

        gz::sim::Model model_{gz::sim::kNullEntity};
        gz::sim::Entity link_entity_{gz::sim::kNullEntity};
        rclcpp::Node::SharedPtr ros_node_;
        std::chrono::steady_clock::duration spawn_time_{0};

        double initial_force_{500.0};
        double force_duration_{0.5};
        double lifetime_{5.0};

    };

}
#endif // STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP

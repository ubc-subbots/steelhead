#ifndef STEELHEAD_GAZEBO__PRESSURE_SENSOR
#define STEELHEAD_GAZEBO__PRESSURE_SENSOR

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include "steelhead_interfaces/msg/pressure_sensor.hpp"
#include <thread>
#include <chrono>

namespace steelhead_gazebo
{
    class PressureSensor : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPostUpdate
    {
    public:
        PressureSensor();
        ~PressureSensor() override;

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm) override;

    private:
        void SpinNode();

        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<steelhead_interfaces::msg::PressureSensor>::SharedPtr pressure_publisher;

        gz::sim::Model model{gz::sim::kNullEntity};
        std::string publish_topic;

        std::thread spinThread;
        int update_rate;
        std::chrono::steady_clock::duration prev_time{0};
    };
}
#endif // STEELHEAD_GAZEBO__PRESSURE_SENSOR

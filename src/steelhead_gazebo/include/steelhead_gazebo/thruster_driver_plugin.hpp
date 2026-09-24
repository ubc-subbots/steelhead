#ifndef STEELHEAD_GAZEBO__THRUSTER_DRIVER_PLUGIN
#define STEELHEAD_GAZEBO__THRUSTER_DRIVER_PLUGIN

#include <vector>
#include <thread>
#include <string>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace steelhead_gazebo
{

    using std::placeholders::_1;

    class ThrusterDriver : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
    {

    public:

        // Constructor
        ThrusterDriver();

        // Destructor
        ~ThrusterDriver() override;

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    private:

        void GetRosNamespace(std::shared_ptr<const sdf::Element> ros_sdf);

        void GetForceCmd(const std_msgs::msg::Float64MultiArray::ConstSharedPtr joint_cmd);

        void SpinNode();

        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd;

        std::vector<gz::sim::Entity> thruster;
        std::vector<double> thrust_values;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
        std::thread spinThread;
        std::string topic_name;

        unsigned int thruster_count;
        
    };

}
#endif // STEELHEAD_GAZEBO__THRUSTER_DRIVER_PLUGIN

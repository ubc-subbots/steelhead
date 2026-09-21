#include "steelhead_gazebo/pressure_sensor.hpp"
#include <gz/sim/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(steelhead_gazebo::PressureSensor,
              gz::sim::System,
              steelhead_gazebo::PressureSensor::ISystemConfigure,
              steelhead_gazebo::PressureSensor::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(steelhead_gazebo::PressureSensor, "steelhead_gazebo::PressureSensor")

namespace steelhead_gazebo
{

    PressureSensor::PressureSensor()
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node = rclcpp::Node::make_shared("pressure_sensor");
    }

    PressureSensor::~PressureSensor()
    {
        if (this->node) rclcpp::shutdown();
        if (this->spinThread.joinable()) this->spinThread.join();
    }

    void PressureSensor::Configure(const gz::sim::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::EventManager &/*_eventMgr*/)
    {
        if (_sdf->HasElement("publish_topic"))
        {
            this->publish_topic = _sdf->Get<std::string>("publish_topic");
        }
        else
        {
            gzerr << "publish_topic value not specified, exiting.\n";
            exit(1);
        }
        if (_sdf->HasElement("update_rate"))
        {
            this->update_rate = _sdf->Get<int>("update_rate");
        }
        else
        {
            gzmsg << "update_rate value not specified, using default: 1Hz.\n";
            this->update_rate = 1;
        }

        this->pressure_publisher = node->create_publisher<steelhead_interfaces::msg::PressureSensor>(this->publish_topic, 10);

        this->model = gz::sim::Model(_entity);

        // CREATE COMPONENT FOR BASE LINK IN CONFIGURE
        gz::sim::Entity linkEntity = this->model.LinkByName(_ecm, "base_link");
        if (linkEntity != gz::sim::kNullEntity) {
            _ecm.CreateComponent(linkEntity, gz::sim::components::WorldPose());
        } else {
            gzerr << "[Pressure Sensor] Could not find base_link inside model!" << std::endl;
        }

        this->spinThread = std::thread(std::bind(&PressureSensor::SpinNode, this));

        this->prev_time = std::chrono::steady_clock::duration::zero();

        gzmsg << "Pressure sensor successfully started!\n";
    }

    void PressureSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                                    const gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        auto current_time = _info.simTime;
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - this->prev_time).count();
        
        if (dt >= (1.0 / this->update_rate))
        {
            this->prev_time = current_time;

            gz::sim::Entity linkEntity = this->model.LinkByName(_ecm, "base_link");
            if (linkEntity != gz::sim::kNullEntity)
            {
                gz::sim::Link link(linkEntity);
                auto pose = link.WorldPose(_ecm);
                if (pose)
                {
                    auto msg = steelhead_interfaces::msg::PressureSensor();
                    msg.depth = -pose->Pos().Z();
                    // mock out the other values because depth is all we are realistically using for now
                    msg.temperature = 20.0; 
                    msg.pressure = 0.0;
                    
                    this->pressure_publisher->publish(msg);
                } else {
                    gzerr << "[Pressure Sensor] WorldPose component is missing!" << std::endl;
                }
            }
        }
    }

    void PressureSensor::SpinNode()
    {
        rclcpp::spin(node);
    }

}

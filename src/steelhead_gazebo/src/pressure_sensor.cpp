#include "steelhead_gazebo/pressure_sensor.hpp"
#include <ignition/math/Pose3.hh>

namespace steelhead_gazebo
{

    PressureSensor::PressureSensor() : node{rclcpp::Node::make_shared("pressure_sensor")} {}


    PressureSensor::~PressureSensor() {}


    void PressureSensor::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

        this->pressure_publisher = node->
            create_publisher<steelhead_interfaces::msg::PressureSensor>(this->publish_topic, 10);

        this->model = _model;
    
        this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                                    std::bind(&PressureSensor::OnUpdate, this));

        this->spinThread = std::thread(std::bind(&PressureSensor::SpinNode, this));

        this->prev_time = node->now();

        gzmsg << "Pressure sensor successfully started!\n";
    }

    void PressureSensor::OnUpdate()
    {
        ignition::math::Pose3d pose = model->WorldPose();
        auto msg = steelhead_interfaces::msg::PressureSensor();
        msg.depth = -pose.Pos()[2];
        msg.temperature = 20; // just mock out some value since depth is the only one of real value as of now
        msg.pressure = 1.0; // see above
        rclcpp::Time now = node->now();
        if ((now- this->prev_time).seconds() >= (1.0/this->update_rate))
        {
            this->prev_time = now;
            this->pressure_publisher->publish(msg);
        }
    }

    void PressureSensor::SpinNode()
    {
        rclcpp::spin(node);
    }

}

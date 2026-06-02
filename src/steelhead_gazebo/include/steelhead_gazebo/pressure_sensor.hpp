#ifndef STEELHEAD_GAZEBO__PRESSURE_SENSOR
#define STEELHEAD_GAZEBO__PRESSURE_SENSOR

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "steelhead_interfaces/msg/pressure_sensor.hpp"

namespace steelhead_gazebo
{

    using std::placeholders::_1;

    class PressureSensor : public gazebo::ModelPlugin
    {

    public:
        PressureSensor(void);
        ~PressureSensor(void);

        /** Collects all neccessary parameters and initializes the ROS 2 node.
         * 
         * @param _model A pointer to the attached mdoel
         * @param _sdf   A pointer to the robot's SDF description
         */
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /** Publishes the PressureSensor message on loop. Assumes that z=0 is the surface of the water.
         * 
         */
        virtual void OnUpdate();

    private:

        /** Spins ROS2 node on a dedicated thread to remain non-blocking
         * 
         */
        void SpinNode(void);

        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<steelhead_interfaces::msg::PressureSensor>::SharedPtr pressure_publisher;
        gazebo::event::ConnectionPtr updateConnection_;
        gazebo::physics::ModelPtr model;
        std::string publish_topic;

        std::thread spinThread;
        std::string topic_name;
        int update_rate;
        rclcpp::Time prev_time;
    };

    GZ_REGISTER_MODEL_PLUGIN(PressureSensor)

}
#endif // STEELHEAD_GAZEBO__PRESSURE_SENSOR

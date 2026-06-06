#ifndef STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP
#define STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"

namespace steelhead_gazebo
{

    class TorpedoPlugin : public gazebo::ModelPlugin
    {

    public:

        // Constructor
        TorpedoPlugin(void);

        // Destructor
        ~TorpedoPlugin(void);

        /** Collects all neccessary parameters and initializes the plugin.
         * * @param _model A pointer to the attached model
         * @param _sdf   A pointer to the model's SDF description
         */
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:

        /** Applies the initial force and checks for lifetime expiration
         * * Updates in sequence with gazebo's main world update function.
         * */
        void OnUpdate(void);

        gazebo::physics::ModelPtr model_;
        gazebo::physics::WorldPtr world_;
        gazebo::physics::LinkPtr link_;
        rclcpp::Node::SharedPtr ros_node_;
        gazebo::common::Time spawn_time_;
        gazebo::event::ConnectionPtr updateConnection_;

        double initial_force_;
        double force_duration_;
        double lifetime_;

    };

    GZ_REGISTER_MODEL_PLUGIN(TorpedoPlugin)

}
#endif // STEELHEAD_GAZEBO__TORPEDO_PLUGIN_HPP
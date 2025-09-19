#ifndef STEELHEAD_EXAMPLE__COMPONENT_TWO
#define STEELHEAD_EXAMPLE__COMPONENT_TWO

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "steelhead_interfaces/msg/pipeline_feedback.hpp"

namespace steelhead_example
{      

    class ComponentTwo : public rclcpp::Node
    {

    public:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        explicit ComponentTwo(const rclcpp::NodeOptions & options);

    private:

        /** Brief description of function.
         * 
         * Longer description in which you describe more in depth about
         * the way the function performs the task mentioned in the brief
         * description above.
         * 
         * @pre precondition of this method, if any
         * 
         * @param options ros2 node options.
         * @param other_param another param
         * 
         * @returns what this function returns, if anything
         * 
         * @post postcondition of this method, if any
         * 
         * @note something of particular note about this function
         * 
         */
        void callback(const std_msgs::msg::String::SharedPtr msg);  

        const int MESSAGE_THRESHOLD = 25;

        int counter_;

        rclcpp::Publisher<steelhead_interfaces::msg::PipelineFeedback>::SharedPtr feedback_pub_; 

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; 

    };
    
} // namespace steelhead_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(steelhead_example::ComponentTwo)

#endif  //STEELHEAD_EXAMPLE__COMPONENT_TWO

#ifndef STEELHEAD_MISSION_PLANNER__MISSION_PLANNER_HPP_
#define STEELHEAD_MISSION_PLANNER__MISSION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <map>
#include <string>
#include <vector>

namespace steelhead_mission_planner
{

/**
 * Configuration for a single state in the decision tree.
 *
 * Each state listens on one completion topic. When "complete" arrives the
 * planner moves to on_complete; when "abort" arrives it moves to on_abort.
 * A value of "undecided" is ignored — the task is still running.
 */
struct StateConfig
{
    std::string listen_topic;  // Topic to read completion strings from
    std::string on_complete;   // Next state when "complete" is received
    std::string on_abort;      // Next state when "abort" is received
};

/**
 * MissionPlanner node
 *
 * Implements a configurable decision-tree mission planner for the Steelhead AUV.
 * The tree is loaded from ROS2 parameters (typically a YAML config file) at
 * startup. Each leaf of the tree is an AUV task state (Gate, Buoy, Torpedo,
 * Octagon, …). The planner:
 *
 *   1. Enters the initial_state and publishes it on /State.
 *   2. Subscribes to the completion topic for that state.
 *   3. Waits for a "complete" or "abort" string (ignores "undecided").
 *   4. Follows the corresponding transition and repeats until "DONE".
 *
 * The /State topic (std_msgs/String) is re-published at 1 Hz so that any
 * node that missed the transition update can always read the current state.
 *
 * This node is completely independent of PipelineSequenceManager.
 */
class MissionPlanner : public rclcpp::Node
{
public:
    /** Constructor — loads the tree from parameters and starts the planner. */
    explicit MissionPlanner(const rclcpp::NodeOptions & options);

private:
    /** Load the decision tree from ROS2 parameters. */
    void load_tree();

    /**
     * Transition the planner into next_state, publish /State, and log.
     * Entering "DONE" stops all further transitions.
     */
    void transition_to(const std::string & next_state);

    /** Publish current_state_ on /State. Called by the 1 Hz timer and on every transition. */
    void publish_state();

    /**
     * Callback fired when any completion topic receives a message.
     *
     * @param topic_name  The topic the message arrived on (captured by lambda).
     * @param msg         The string payload ("undecided" | "complete" | "abort").
     */
    void on_completion_message(
        const std::string & topic_name,
        const std_msgs::msg::String::SharedPtr msg);

    // ── State machine ──────────────────────────────────────────────────────
    std::string current_state_;
    std::string initial_state_;
    std::map<std::string, StateConfig> state_tree_;

    // ── ROS interfaces ─────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    // All completion subscriptions (one per state, kept alive for node lifetime)
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
};

}  // namespace steelhead_mission_planner

#endif  // STEELHEAD_MISSION_PLANNER__MISSION_PLANNER_HPP_

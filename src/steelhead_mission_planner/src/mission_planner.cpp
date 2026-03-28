#include "steelhead_mission_planner/mission_planner.hpp"

#include <chrono>
#include <functional>

namespace steelhead_mission_planner
{

// ── Constructor ────────────────────────────────────────────────────────────────

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options)
{
    load_tree();

    // Publisher: broadcast current mission state at 1 Hz (and on every transition)
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/State", 10);

    // Subscribe to every completion topic defined in the tree.
    // All subscriptions are created up-front; on_completion_message filters
    // out topics that do not belong to the currently active state.
    for (const auto & [state_name, cfg] : state_tree_) {
        const std::string topic = cfg.listen_topic;

        auto sub = this->create_subscription<std_msgs::msg::String>(
            topic, 10,
            [this, topic](const std_msgs::msg::String::SharedPtr msg) {
                on_completion_message(topic, msg);
            });

        subscriptions_.push_back(sub);
        RCLCPP_INFO(this->get_logger(),
            "Subscribed to completion topic '%s' for state '%s'.",
            topic.c_str(), state_name.c_str());
    }

    // Periodic state broadcast so late subscribers never miss the current state
    state_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MissionPlanner::publish_state, this));

    // Enter initial state
    current_state_ = initial_state_;
    RCLCPP_INFO(this->get_logger(),
        "Mission Planner started. Initial state: '%s'.", current_state_.c_str());
    publish_state();
}

// ── Private methods ────────────────────────────────────────────────────────────

void MissionPlanner::load_tree()
{
    // --- Required parameters ---
    this->declare_parameter<std::string>("initial_state", "Gate");
    this->declare_parameter<std::vector<std::string>>("states", {});

    initial_state_       = this->get_parameter("initial_state").as_string();
    auto state_names     = this->get_parameter("states").as_string_array();

    if (state_names.empty()) {
        RCLCPP_ERROR(this->get_logger(),
            "Parameter 'states' is empty. Check your mission_tree.yaml config.");
        return;
    }

    // --- Per-state parameters ---
    for (const auto & state : state_names) {
        this->declare_parameter<std::string>(state + ".listen_topic", "");
        this->declare_parameter<std::string>(state + ".on_complete",  "DONE");
        this->declare_parameter<std::string>(state + ".on_abort",     "DONE");

        StateConfig cfg;
        cfg.listen_topic = this->get_parameter(state + ".listen_topic").as_string();
        cfg.on_complete  = this->get_parameter(state + ".on_complete").as_string();
        cfg.on_abort     = this->get_parameter(state + ".on_abort").as_string();

        if (cfg.listen_topic.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "State '%s' has no listen_topic configured.", state.c_str());
        }

        state_tree_[state] = cfg;

        RCLCPP_INFO(this->get_logger(),
            "Loaded state '%s': topic='%s', complete->'%s', abort->'%s'.",
            state.c_str(),
            cfg.listen_topic.c_str(),
            cfg.on_complete.c_str(),
            cfg.on_abort.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
        "Decision tree loaded: %zu state(s), initial='%s'.",
        state_tree_.size(), initial_state_.c_str());
}

void MissionPlanner::publish_state()
{
    auto msg = std_msgs::msg::String();
    msg.data = current_state_;
    state_publisher_->publish(msg);
}

void MissionPlanner::on_completion_message(
    const std::string & topic_name,
    const std_msgs::msg::String::SharedPtr msg)
{
    // Nothing to do once the mission is finished
    if (current_state_ == "DONE") {
        return;
    }

    // Retrieve config for the current state
    auto it = state_tree_.find(current_state_);
    if (it == state_tree_.end()) {
        RCLCPP_ERROR(this->get_logger(),
            "Current state '%s' is not in the decision tree.", current_state_.c_str());
        return;
    }

    // Only process messages that belong to the currently active state's topic
    if (it->second.listen_topic != topic_name) {
        return;
    }

    const std::string & value = msg->data;

    if (value == "undecided") {
        // Task is still running — nothing to act on
        return;
    }

    if (value == "complete") {
        RCLCPP_INFO(this->get_logger(),
            "State '%s' reported COMPLETE. Transitioning to '%s'.",
            current_state_.c_str(), it->second.on_complete.c_str());
        transition_to(it->second.on_complete);

    } else if (value == "abort") {
        RCLCPP_WARN(this->get_logger(),
            "State '%s' reported ABORT. Transitioning to '%s'.",
            current_state_.c_str(), it->second.on_abort.c_str());
        transition_to(it->second.on_abort);

    } else {
        RCLCPP_WARN(this->get_logger(),
            "Unknown completion value '%s' on topic '%s'. "
            "Expected 'complete', 'abort', or 'undecided'.",
            value.c_str(), topic_name.c_str());
    }
}

void MissionPlanner::transition_to(const std::string & next_state)
{
    current_state_ = next_state;
    publish_state();

    if (current_state_ == "DONE") {
        RCLCPP_INFO(this->get_logger(), "All tasks complete. Mission is DONE.");
        return;
    }

    if (state_tree_.find(current_state_) == state_tree_.end()) {
        RCLCPP_ERROR(this->get_logger(),
            "Transition target '%s' is not defined in the mission tree. "
            "Check your mission_tree.yaml config.",
            current_state_.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "Now in state '%s'. Listening on '%s'.",
        current_state_.c_str(),
        state_tree_.at(current_state_).listen_topic.c_str());
}

}  // namespace steelhead_mission_planner

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<steelhead_mission_planner::MissionPlanner>(options));
    rclcpp::shutdown();
    return 0;
}

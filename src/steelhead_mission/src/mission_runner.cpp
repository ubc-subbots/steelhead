#include "steelhead_mission/mission_runner.hpp"

#include <chrono>
#include <filesystem>
#include <sstream>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"

#include "steelhead_mission/tasks/register_nodes.hpp"


namespace mission
{


    MissionRunner::MissionRunner(const rclcpp::NodeOptions & options)
    : Node("mission_runner", options)
    {
        this->declare_parameter<std::string>("mission_file", "gate_only.xml");
        this->declare_parameter<double>("tick_rate_hz", 20.0);
        this->declare_parameter<int>("groot_port", 1667);

        mission_file_ = this->get_parameter("mission_file").as_string();
        tick_rate_hz_ = this->get_parameter("tick_rate_hz").as_double();
        groot_port_ = static_cast<int>(this->get_parameter("groot_port").as_int());

        state_pub_ = this->create_publisher<std_msgs::msg::String>("mission/state", 10);
        hover_adjust_pub_ = this->create_publisher<steelhead_interfaces::msg::HoverAdjustment>(
            "controls/hover_adjust", 10
        );
    }


    int MissionRunner::run()
    {
        registerTaskNodes(factory_, this->shared_from_this());

        const std::string xml_path = resolveMissionPath(mission_file_);
        RCLCPP_INFO(
            this->get_logger(),
            "Loading behavior tree from '%s' (tick_rate=%.1f Hz, groot_port=%d)",
            xml_path.c_str(), tick_rate_hz_, groot_port_
        );

        try
        {
            tree_ = factory_.createTreeFromFile(xml_path);
        }
        catch (const std::exception & ex)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load behavior tree '%s': %s",
                xml_path.c_str(), ex.what()
            );
            return 1;
        }

        cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
        if (groot_port_ > 0)
        {
            try
            {
                groot_publisher_ = std::make_unique<BT::Groot2Publisher>(
                    tree_, static_cast<unsigned>(groot_port_)
                );
                RCLCPP_INFO(
                    this->get_logger(),
                    "Groot2 publisher listening on TCP port %d",
                    groot_port_
                );
            }
            catch (const std::exception & ex)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Could not start Groot2Publisher on port %d: %s",
                    groot_port_, ex.what()
                );
            }
        }

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->shared_from_this());

        rclcpp::WallRate rate(tick_rate_hz_ > 0.0 ? tick_rate_hz_ : 20.0);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        std::string last_status_text;

        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
        {
            executor.spin_some();
            status = tree_.tickExactlyOnce();

            std::string status_text = formatActiveStatus(status);
            if (status_text != last_status_text)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Mission status: %s",
                    status_text.c_str()
                );
                last_status_text = status_text;
            }
            publishStatus(status_text);

            rate.sleep();
        }

        if (status == BT::NodeStatus::RUNNING)
        {
            tree_.haltTree();
        }
        publishZeroWrench();

        const std::string final_text = formatActiveStatus(status);
        publishStatus(final_text);
        RCLCPP_INFO(
            this->get_logger(),
            "Mission finished with status: %s",
            BT::toStr(status).c_str()
        );

        return status == BT::NodeStatus::SUCCESS ? 0 : 1;
    }


    std::string MissionRunner::resolveMissionPath(const std::string & mission_file) const
    {
        std::filesystem::path candidate(mission_file);
        if (candidate.is_absolute() || std::filesystem::exists(candidate))
        {
            return candidate.string();
        }

        std::filesystem::path share_dir =
            ament_index_cpp::get_package_share_path("steelhead_mission");
        return (share_dir / "missions" / candidate).string();
    }


    std::string MissionRunner::formatActiveStatus(BT::NodeStatus root_status) const
    {
        if (root_status == BT::NodeStatus::SUCCESS)
        {
            return "DONE [SUCCESS]";
        }
        if (root_status == BT::NodeStatus::FAILURE)
        {
            return "ABORTED [FAILURE]";
        }

        std::vector<std::string> active_leaves;
        for (const auto & subtree : tree_.subtrees)
        {
            for (const auto & node : subtree->nodes)
            {
                if (node->type() == BT::NodeType::ACTION &&
                    node->status() == BT::NodeStatus::RUNNING)
                {
                    if (node->name() != node->registrationName())
                    {
                        active_leaves.push_back(
                            node->name() + " (" + node->registrationName() + ")"
                        );
                    }
                    else
                    {
                        active_leaves.push_back(node->name());
                    }
                }
            }
        }

        if (active_leaves.empty())
        {
            return "RUNNING";
        }

        std::ostringstream oss;
        oss << "RUNNING: ";
        for (size_t i = 0; i < active_leaves.size(); ++i)
        {
            if (i > 0)
            {
                oss << ", ";
            }
            oss << active_leaves[i];
        }
        return oss.str();
    }


    void MissionRunner::publishStatus(const std::string & status_text)
    {
        auto msg = std_msgs::msg::String();
        msg.data = status_text;
        state_pub_->publish(msg);
    }


    // TODO: This is tightly coupled with the hoveradjustment node, which needs
    // a zero publish. Remove when reworking the hoveradjument node.
    void MissionRunner::publishZeroWrench()
    {
        auto zero = steelhead_interfaces::msg::HoverAdjustment();
        zero.type = steelhead_interfaces::msg::HoverAdjustment::PARTIAL;
        hover_adjust_pub_->publish(zero);
    }


} // namespace mission


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto runner = std::make_shared<mission::MissionRunner>();
    int exit_code = runner->run();
    rclcpp::shutdown();
    return exit_code;
}

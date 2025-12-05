#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node
{
public:
    LifecycleManager() : Node("lifecycle_manager")
    {
        // Get namespace parameter to manage nodes in specific robot
        this->declare_parameter("managed_nodes", std::vector<std::string>{"node1", "node2", "node3", "node4"});
        managed_nodes_ = this->get_parameter("managed_nodes").as_string_array();

        RCLCPP_INFO(this->get_logger(), "Lifecycle Manager started");

        // Wait a bit for lifecycle nodes to be ready
        rclcpp::sleep_for(1s);

        // Configure and activate all managed nodes
        configure_and_activate_all();
    }

private:
    void configure_and_activate_all()
    {
        for (const auto& node_name : managed_nodes_) {
            std::string full_node_name = this->get_namespace() + std::string("/") + node_name;

            RCLCPP_INFO(this->get_logger(), "Managing lifecycle node: %s", full_node_name.c_str());

            // Configure the node
            if (change_state(full_node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
                RCLCPP_INFO(this->get_logger(), "Configured: %s", full_node_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure: %s", full_node_name.c_str());
                continue;
            }

            // Activate the node
            if (change_state(full_node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                RCLCPP_INFO(this->get_logger(), "Activated: %s", full_node_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to activate: %s", full_node_name.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "All managed nodes configured and activated");
    }

    bool change_state(const std::string& node_name, uint8_t transition_id)
    {
        std::string service_name = node_name + "/change_state";
        auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

        // Wait for service to be available
        if (!client->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return false;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        auto future = client->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future.get();
            return result->success;
        }

        RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", service_name.c_str());
        return false;
    }

    std::vector<std::string> managed_nodes_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto manager = std::make_shared<LifecycleManager>();
    rclcpp::spin(manager);
    rclcpp::shutdown();
    return 0;
}

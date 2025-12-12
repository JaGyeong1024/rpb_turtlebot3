#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Node2 : public rclcpp_lifecycle::LifecycleNode
{
public:
    Node2() : LifecycleNode("node2"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 lifecycle node created");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 configuring...");
        publisher_ = this->create_publisher<std_msgs::msg::String>("pub2", 10);
        RCLCPP_INFO(this->get_logger(), "Node2 configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 activating...");
        publisher_->on_activate();
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&Node2::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Node2 activated, publishing to pub2");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 deactivating...");
        timer_->cancel();
        publisher_->on_deactivate();
        RCLCPP_INFO(this->get_logger(), "Node2 deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 cleaning up...");
        timer_.reset();
        publisher_.reset();
        RCLCPP_INFO(this->get_logger(), "Node2 cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Node2 shutting down...");
        timer_.reset();
        publisher_.reset();
        RCLCPP_INFO(this->get_logger(), "Node2 shutdown complete");
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from Node2: " + std::to_string(count_);
        RCLCPP_INFO(this->get_logger(), "Publishing to pub2: '%s'", message.data.c_str());
        publisher_->publish(message);
        count_++;
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node2>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

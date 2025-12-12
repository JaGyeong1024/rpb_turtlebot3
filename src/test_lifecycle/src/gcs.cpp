#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GCSNode : public rclcpp::Node
{
public:
    GCSNode() : Node("gcs")
    {
        // Subscribe to pub3 from all 3 robots
        for (int robot_id = 1; robot_id <= 3; robot_id++) {
            std::string topic = "/robot" + std::to_string(robot_id) + "/pub3";
            auto sub = this->create_subscription<std_msgs::msg::String>(
                topic, 10,
                [this, robot_id](const std_msgs::msg::String::SharedPtr msg) {
                    this->topic_callback(robot_id, 3, msg);
                });
            subscribers_.push_back(sub);

            RCLCPP_INFO(this->get_logger(), "GCS subscribing to %s", topic.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "GCS Node started - monitoring node3 from all robots");
    }

private:
    void topic_callback(int robot_id, int node_num, const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "[GCS] Robot%d/Node%d: '%s'",
                    robot_id, node_num, msg->data.c_str());
    }

    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GCSNode>());
    rclcpp::shutdown();
    return 0;
}

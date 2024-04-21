#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ForcePublisherNode : public rclcpp::Node
{
public:
    ForcePublisherNode() : Node("force_publisher_node")
    {
        // 센서 데이터 subscriber 생성
        sensor_sub_1 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data_1", 10, std::bind(&ForcePublisherNode::sensor_callback_1, this, std::placeholders::_1));
        sensor_sub_2 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data_2", 10, std::bind(&ForcePublisherNode::sensor_callback_2, this, std::placeholders::_1));
        sensor_sub_3 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data_3", 10, std::bind(&ForcePublisherNode::sensor_callback_3, this, std::placeholders::_1));
        sensor_sub_4 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data_4", 10, std::bind(&ForcePublisherNode::sensor_callback_4, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ForcePublisherNode::publish_force, this));
        force_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("external_force", rclcpp::SystemDefaultsQoS());
    }

private:
    void sensor_callback_1(const std_msgs::msg::Int32::SharedPtr msg)
    {
        force_sensor[0] = msg->data;
    }

    void sensor_callback_2(const std_msgs::msg::Int32::SharedPtr msg)
    {
        force_sensor[1] = msg->data;
    }

    void sensor_callback_3(const std_msgs::msg::Int32::SharedPtr msg)
    {
        force_sensor[2] = msg->data;
    }

    void sensor_callback_4(const std_msgs::msg::Int32::SharedPtr msg)
    {
        force_sensor[3] = msg->data;
    }

    void publish_force()
    {
        double x_force = (force_sensor[0] + force_sensor[1]) - (force_sensor[2] + force_sensor[3]);
        double yaw_force = (force_sensor[1] - force_sensor[0]) * K;

        std_msgs::msg::Float64MultiArray force_msg;
        force_msg.data = {x_force, 0, yaw_force};

        force_pub->publish(force_msg);
    }

    double force_sensor[4] = {0.0, 0.0, 0.0, 0.0};
    double K = 1.0;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_2;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_3;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_4;

    //Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForcePublisherNode>());
    rclcpp::shutdown();
    return 0;
}

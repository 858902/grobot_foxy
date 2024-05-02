#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <deque>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class ForcePublisherNode : public rclcpp::Node
{
public:
    ForcePublisherNode() : Node("force_publisher_node")
    {
        // 센서 데이터 subscriber 생성
        sensor_sub_1 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data/sensor1", 10, std::bind(&ForcePublisherNode::sensor_callback_1, this, std::placeholders::_1));
        sensor_sub_2 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data/sensor2", 10, std::bind(&ForcePublisherNode::sensor_callback_2, this, std::placeholders::_1));
        sensor_sub_3 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data/sensor3", 10, std::bind(&ForcePublisherNode::sensor_callback_3, this, std::placeholders::_1));
        sensor_sub_4 = this->create_subscription<std_msgs::msg::Int32>(
            "sensor_data/sensor4", 10, std::bind(&ForcePublisherNode::sensor_callback_4, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ForcePublisherNode::publish_force, this));
        force_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("external_force", rclcpp::SystemDefaultsQoS());
    }

private:
    void sensor_callback_1(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // force_sensor[0] = msg->data;
        // update_moving_average(0, msg->data);
        // update_low_pass_filter(0, msg->data);
        update_moving_rms(0, msg->data);
    }

    void sensor_callback_2(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // force_sensor[1] = msg->data;
        // update_moving_average(1, msg->data);
        // update_low_pass_filter(1, msg->data);
        update_moving_rms(1, msg->data);

    }

    void sensor_callback_3(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // force_sensor[2] = msg->data;
        // update_moving_average(2, msg->data);
        // update_low_pass_filter(2, msg->data);
        update_moving_rms(2, msg->data);


    }

    void sensor_callback_4(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // force_sensor[3] = msg->data;
        // update_moving_average(3, msg->data);
        // update_low_pass_filter(3, msg->data);
        update_moving_rms(3, msg->data);


    }

    void publish_force()
    {
        double x_force = ((force_sensor[0] + force_sensor[1]) - (force_sensor[2] + force_sensor[3])) * K_x;
        double yaw_force = (force_sensor[1] - force_sensor[0]) * K_yaw;

        // 외력 upper limit 설정 
        x_force = std::clamp(x_force, -0.4, 0.4); 
        yaw_force = std::clamp(yaw_force, -0.1, 0.1); 

        std_msgs::msg::Float64MultiArray force_msg;
        force_msg.data = {x_force, 0, yaw_force};

        force_pub->publish(force_msg);
    }

    // Moving Average Fillter 
    void update_moving_average(int index, int new_data)
    {
        sensor_data[index].push_back(new_data);
        if (sensor_data[index].size() > N)
            sensor_data[index].pop_front();
        
        force_sensor[index] = std::accumulate(sensor_data[index].begin(), sensor_data[index].end(), 0.0) / sensor_data[index].size();
    }

    // Low-Pass Filter
    void update_low_pass_filter(int index, int new_data)
    {
        // Apply the low-pass filter
        force_sensor[index] = alpha * new_data + (1.0 - alpha) * force_sensor[index];
    }
    // Moving RMS Filter
    void update_moving_rms(int index, int new_data)
    {
        if (sensor_data[index].size() >= N)
        {
            sensor_data[index].pop_front();
        }
        sensor_data[index].push_back(pow(new_data, 2)); //제곱 값 저장
        
        double sum = std::accumulate(sensor_data[index].begin(), sensor_data[index].end(), 0.0);
        force_sensor[index] = sqrt(sum / sensor_data[index].size()); // RMS 계산
    }

    static constexpr double alpha = 0.1; // 저주파 필터의 감쇠 계수
    static constexpr size_t N = 50; //moving size 
    // std::deque<int> sensor_data[4];
    std::deque<double> sensor_data[4];

    double force_sensor[4] = {0.0, 0.0, 0.0, 0.0};
    double K_x = 0.005;
    double K_yaw = 0.002;


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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <deque>
#include <algorithm>
#include <cmath>
#include <string>
#include "std_msgs/msg/string.hpp"

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

        offset_sub = this->create_subscription<std_msgs::msg::String>(
            "signal_offset", 10, std::bind(&ForcePublisherNode::signal_offset_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ForcePublisherNode::publish_force, this));
        force_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("external_force", rclcpp::SystemDefaultsQoS());
    }

private:

    void signal_offset_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "start")
        {   
            std::cout << "signal"<< std::endl;

            // 오프셋 측정 시작
            std::fill(std::begin(initial_force_sum_), std::end(initial_force_sum_), 0);
            std::fill(std::begin(initial_samples_collected_), std::end(initial_samples_collected_), 0);
            offset_initialized_ = false;
        }
        else if (msg->data == "end" && !offset_initialized_)
        {
            // 오프셋 측정 완료 및 초기화
            for (int i = 0; i < 4; i++)
            {
                if (initial_samples_collected_[i] > 0)
                {
                    offset_[i] = initial_force_sum_[i] / initial_samples_collected_[i];
                }
            }
            std::cout << "offset vector :"<< offset_[0] << " " << offset_[1] << " " << offset_[2] << " " << offset_[3]<< std::endl;
            offset_initialized_ = true;
        }
    }

    void sensor_callback_1(const std_msgs::msg::Int32::SharedPtr msg)
    {   
        if (!offset_initialized_)
        {
            initial_force_sum_[0] += msg->data * K_x;
            initial_samples_collected_[0]++;
        }

        else
        {
            double adjusted_data_0 = (msg->data * K_x) - offset_[0];
            adjusted_data_0 = std::clamp(adjusted_data_0, 0.0, 100.0); 
            // update_moving_rms(0, adjusted_data_0);
            force_sensor[0] = adjusted_data_0;
            // update_moving_average(0, msg->data);
            // update_low_pass_filter(0, msg->data);
            // update_moving_rms(0, msg->data);
        }
    }

    void sensor_callback_2(const std_msgs::msg::Int32::SharedPtr msg)
    {   
        if (!offset_initialized_)
        {
            initial_force_sum_[1] += msg->data * K_x;
            initial_samples_collected_[1]++;
        }

        else
        {
            double adjusted_data_1 = (msg->data * K_x) - offset_[1];
            adjusted_data_1 = std::clamp(adjusted_data_1, 0.0, 100.0); 
            // std::cout << " adjusted_data_1 :"<< adjusted_data_1<< std::endl;
            // update_moving_rms(1, adjusted_data_1);
            force_sensor[1] = adjusted_data_1;
            // update_moving_average(1, msg->data);
            // update_low_pass_filter(1, msg->data);
            // update_moving_rms(1, msg->data);
        }
    }

    void sensor_callback_3(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!offset_initialized_)
        {
            initial_force_sum_[2] += msg->data * K_x;
            initial_samples_collected_[2]++;
        }

        else
        {
            double adjusted_data_2 = msg->data * K_x - offset_[2];
            adjusted_data_2 = std::clamp(adjusted_data_2, 0.0, 100.0); 
            // update_moving_rms(2, adjusted_data_2);
            force_sensor[2] = adjusted_data_2;
            // update_moving_average(2, msg->data);
            // update_low_pass_filter(2, msg->data);
            // update_moving_rms(2, msg->data);
        }
    }

    void sensor_callback_4(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!offset_initialized_)
        {
            initial_force_sum_[3] += msg->data * K_x;
            initial_samples_collected_[3]++;
        }

        else
        {
            double adjusted_data_3 = (msg->data * K_x) - offset_[3];
            adjusted_data_3 = std::clamp(adjusted_data_3, 0.0, 100.0); 
            // update_moving_rms(3, adjusted_data_3);
            force_sensor[3] = adjusted_data_3;
            // update_moving_average(3, msg->data);
            // update_low_pass_filter(3, msg->data);
            // update_moving_rms(3, msg->data);
        }
    }

    void publish_force()
    {
        if (offset_initialized_)
        {
            double x_force = ((force_sensor[0] + force_sensor[1]) - (force_sensor[2] + force_sensor[3]));
            double yaw_force = (force_sensor[1] - force_sensor[0]);

            // x_force = std::exp(x_force * 0.5) - 1;
            
            // 외력 upper limit 설정 
            x_force = std::clamp(x_force, -0.4, 0.5); 
            yaw_force = std::clamp(yaw_force, -0.1, 0.1); 

            // //Dead Zone
            // if (x_force > -0.08 && x_force < 0.08)
            // {
            //     x_force = 0;
            // }

            std_msgs::msg::Float64MultiArray force_msg;
            force_msg.data = {x_force, 0, yaw_force};

            force_pub->publish(force_msg);
        }
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
        force_sensor[index] = alpha * new_data + (1.0 - alpha) * force_sensor[index];
    }

    // Moving RMS Filter
    void update_moving_rms(int index, double new_data)
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
    bool offset_initialized_ = false;
    double offset_[4] = {0.0, 0.0, 0.0, 0.0};
    double initial_force_sum_[4] = {0.0, 0.0, 0.0, 0.0};
    double initial_samples_collected_[4] = {0.0, 0.0, 0.0, 0.0};


    double force_sensor[4] = {0.0, 0.0, 0.0, 0.0};
    double K_x = 0.0015;
    double K_yaw = 0.002;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_2;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_3;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_4;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offset_sub;

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

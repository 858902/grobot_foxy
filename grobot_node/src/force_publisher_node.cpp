#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <deque>
#include <algorithm>
#include <cmath>
#include <string>
#include "std_msgs/msg/string.hpp"
#include <numeric>
#include <memory>

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
        plot_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("filter_check", rclcpp::SystemDefaultsQoS());

        compute_gaussian_weights(); 

    }

private:

    void signal_offset_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "start")
        {   
            std::cout << "offset_check_start"<< std::endl;

            // 오프셋 측정 시작
            std::fill(std::begin(initial_force_sum_), std::end(initial_force_sum_), 0);
            std::fill(std::begin(initial_samples_collected_), std::end(initial_samples_collected_), 0);
            offset_initialized_ = false;
        }
        else if (msg->data == "end" && !offset_initialized_)
        {   
            std::cout << "offset_check_done"<< std::endl;

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
            initial_force_sum_[0] += msg->data;
            initial_samples_collected_[0]++;
        }

        else
        {
            double adjusted_data_0 = (msg->data - offset_[0]) * scale_gain_;
            adjusted_data_0 = std::clamp(adjusted_data_0, 0.0, 100.0); 
            update_gaussian_filter(0, adjusted_data_0);
            // force_sensor[0] = adjusted_data_0;
            // update_moving_average(0, adjusted_data_0);
            // update_low_pass_filter(0, msg->data);
        }
    }

    void sensor_callback_2(const std_msgs::msg::Int32::SharedPtr msg)
    {   
        if (!offset_initialized_)
        {
            initial_force_sum_[1] += msg->data;
            initial_samples_collected_[1]++;
        }

        else
        {
            double adjusted_data_1 = (msg->data - offset_[1])* scale_gain_;
            adjusted_data_1 = std::clamp(adjusted_data_1, 0.0, 100.0); 
            update_gaussian_filter(1, adjusted_data_1);
            // force_sensor[1] = adjusted_data_1;
            // update_moving_average(1, adjusted_data_1);
            // update_low_pass_filter(1, msg->data);
        }
    }

    void sensor_callback_3(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!offset_initialized_)
        {
            initial_force_sum_[2] += msg->data;
            initial_samples_collected_[2]++;
        }

        else
        {
            double adjusted_data_2 = (msg->data - offset_[2]) * scale_gain_2;
            adjusted_data_2 = std::clamp(adjusted_data_2, 0.0, 100.0); 
            update_gaussian_filter(2, adjusted_data_2);
            // force_sensor[2] = adjusted_data_2;
            // update_moving_average(2, adjusted_data_2);
            // update_low_pass_filter(2, msg->data);
        }
    }

    void sensor_callback_4(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!offset_initialized_)
        {
            initial_force_sum_[3] += msg->data;
            initial_samples_collected_[3]++;
        }

        else
        {
            double adjusted_data_3 = (msg->data - offset_[3]) * scale_gain_2;
            adjusted_data_3 = std::clamp(adjusted_data_3, 0.0, 100.0); 
            update_gaussian_filter(3, adjusted_data_3);
            // force_sensor[3] = adjusted_data_3;
            // update_moving_average(3, adjusted_data_3);
            // update_low_pass_filter(3, msg->data);
        }
    }

    void publish_force()
    {
        if (offset_initialized_)
        {   
            std_msgs::msg::Float64MultiArray msg_;
            msg_.data = {force_sensor[0], force_sensor[1], force_sensor[2],force_sensor[3]};
            plot_pub->publish(msg_);

            double x_force = ((force_sensor[0] + force_sensor[1]) - (force_sensor[2] + force_sensor[3]));
            double yaw_force = (force_sensor[1]- force_sensor[0]) * scale_gain_yaw;
            // double yaw_force = (force_sensor[1] +force_sensor[2])- (force_sensor[0] + force_sensor[3]) * scale_gain_yaw;
            
            // 외력 upper & under limit 설정 
            x_force = std::clamp(x_force, -0.5, 0.5); 
            yaw_force = std::clamp(yaw_force, -1.0, 1.0); 

            // //Dead Zone
            if (x_force > -0.07 && x_force < 0.07)
            {
                x_force = 0;
            }

            if (yaw_force > -0.01 && yaw_force < 0.01)
            {
                yaw_force = 0;
            }

            // update_yaw_force_moving_average(yaw_force);

            std_msgs::msg::Float64MultiArray force_msg;
            force_msg.data = {x_force, 0, yaw_force};

            force_pub->publish(force_msg);
        }
    }

    // Moving Average Fillter 
    static constexpr size_t N = 10; //moving size 

    void update_moving_average(int index, double new_data)
    {
        sensor_data[index].push_back(new_data);
        if (sensor_data[index].size() > N)
            sensor_data[index].pop_front();
        
        force_sensor[index] = std::accumulate(sensor_data[index].begin(), sensor_data[index].end(), 0.0) / sensor_data[index].size();
    }

    // Low-Pass Filter
    static constexpr double alpha = 0.1;

    void update_low_pass_filter(int index, int new_data)
    {
        force_sensor[index] = alpha * new_data + (1.0 - alpha) * force_sensor[index];
    }

    // Gaussian Filter
    double sigma = 50.0; 
    const size_t gaussian_size = 151;  // Kernal Size
    std::vector<double> gaussian_weights;

    void compute_gaussian_weights() 
    {   

        gaussian_weights.resize(gaussian_size);
        double sum = 0.0;
        int half_size = gaussian_size / 2;
        for (int i = 0; i < gaussian_size; ++i) 
        {
            int x = i - half_size;
            gaussian_weights[i] = std::exp(-0.5 * (x * x) / (sigma * sigma)) / (sigma * std::sqrt(2.0 * M_PI));
            sum += gaussian_weights[i];
        }

        // 가중치 정규화
        for (double& weight : gaussian_weights) 
        {
            weight /= sum;
        }

        // for (const double& weight : gaussian_weights)
        // {
        //     std::cout << weight << " ";
        // }
        // std::cout << std::endl;
    }

    void update_gaussian_filter(int index, double new_data)
     {
        sensor_data[index].push_back(new_data);
        if (sensor_data[index].size() > gaussian_size)
            sensor_data[index].pop_front();
        
        if (sensor_data[index].size() == gaussian_size)
        {
            double filtered_value = 0.0;
            for (size_t i = 0; i < gaussian_size; ++i)
            {
                filtered_value += sensor_data[index][i] * gaussian_weights[i];
            }
            force_sensor[index] = filtered_value;
        }
    }

    // Offset 
    std::deque<double> sensor_data[4];
    bool offset_initialized_ = false;
    double offset_[4] = {0.0, 0.0, 0.0, 0.0};
    double initial_force_sum_[4] = {0.0, 0.0, 0.0, 0.0};
    double initial_samples_collected_[4] = {0.0, 0.0, 0.0, 0.0};

    double force_sensor[4] = {0.0, 0.0, 0.0, 0.0};

    // double scale_gain_ = 0.0025;
    double scale_gain_ = 0.003;
    double scale_gain_2 = 0.005; //0.005
    double scale_gain_yaw = 0.2;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_2;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_3;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_4;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offset_sub;

    //Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr plot_pub;


    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForcePublisherNode>());
    rclcpp::shutdown();
    return 0;
}

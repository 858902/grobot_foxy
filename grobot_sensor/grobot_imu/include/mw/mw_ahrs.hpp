#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <iostream>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "MW_serial.hpp"

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35
#define MAG 0x36

#define convertor_g2a 9.80665        // linear_acceleration (g to m/s^2)
#define convertor_d2r (M_PI / 180.0) // angular_velocity (degree to radian)
#define convertor_ut2t 1000000       // magnetic_field (uT to Tesla)
#define convertor_c 1.0              // temperature (celsius)

using namespace std::chrono_literals;

static float acc_value[3] = {
    0,
},
             gyr_value[3] = {
                 0,
},
             deg_value[3] = {
                 0,
},
             mag_value[3] = {
                 0,
};
static bool run_bool = false;

namespace ntrex
{
    class MwAhrsRosDriver : public rclcpp::Node
    {
    public:
    private:
        pid_t pid;
        pthread_t tid;

        bool publish_tf_;
        std::string parent_frame_id_;
        std::string frame_id_;

        double linear_acceleration_stddev_;
        double angular_velocity_stddev_;
        double magnetic_field_stddev_;
        double orientation_stddev_;

    public:
        MwAhrsRosDriver(char *port, int baud_rate, int sel);
        ~MwAhrsRosDriver();

        static void *MwAhrsRead(void *);
        void publish_topic();
        tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw);

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_, imu_data_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_yaw_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

        // ROS timer
        rclcpp::TimerBase::SharedPtr AHRS_publish_topic_timer;
    };
}
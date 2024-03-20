#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>
#include <vector>
#include <unordered_map>

#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>

using namespace std;

class LidarCalibrationNode : public rclcpp::Node
{
public:
  LidarCalibrationNode()
      : Node("lidar_calibration_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    
    // Subscriber
    subscription_lidar_right =  this->create_subscription<sensor_msgs::msg::LaserScan>(
        "LIDAR1/scan", 10, std::bind(&LidarCalibrationNode::scan_callback_right, this, std::placeholders::_1));

    subscription_lidar_left =  this->create_subscription<sensor_msgs::msg::LaserScan>(
        "LIDAR2/scan", 10, std::bind(&LidarCalibrationNode::scan_callback_left, this, std::placeholders::_1));
    
    //Publisher
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    scan_pub1 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan1", 10);
    scan_pub2 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 10);
    scan_pub3 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan3", 10);
    scan_pub4 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan4", 10);
  }

  void scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    // laser1_ = msg;
    // publish_filtered_scan(msg, 0, 180, scan_pub1); 
    // publish_filtered_scan(msg, -180, -90, scan_pub2); 
    // if(laser2_)
    // {
    update_pointcloud(msg, 0, 180, "base_scan",true);
    // }

  }

  
  void scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    // laser2_ = msg;
    // publish_filtered_scan(msg, -180, 0, scan_pub3); 
    // publish_filtered_scan(msg, 90, 180, scan_pub4); 
    update_pointcloud(msg, -180, 0, "base_scan2",false);

  }
    
  void publish_filtered_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_min_deg, double angle_max_deg, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub) 
  {
    auto filtered_scan = *msg;
    filtered_scan.ranges.clear();
    filtered_scan.ranges.resize(msg->ranges.size(), std::numeric_limits<float>::infinity());

    double angle_min = angle_min_deg * M_PI / 180.0;
    double angle_max = angle_max_deg * M_PI / 180.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) 
    {
        double angle = msg->angle_min + i * msg->angle_increment;
        if (angle >= angle_min && angle <= angle_max) 
        {
            filtered_scan.ranges[i] = msg->ranges[i];
        }
    }

    pub->publish(filtered_scan);
  }

  void update_pointcloud(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_min_deg_, double angle_max_deg_, std::string frame_id, bool isRight) 
  {
        auto integradted_scan = *msg;

        // sensor_msgs::msg::LaserScan integradted_scan;
        integradted_scan.header.frame_id = frame_id;
        integradted_scan.header.stamp = msg->header.stamp; 
        // integradted_scan.header.stamp = this->get_clock()->now();
        integradted_scan.ranges.clear();
        integradted_scan.ranges.resize(msg->ranges.size(), std::numeric_limits<float>::infinity());

        if (frame_id != "base_scan") 
        {   
          geometry_msgs::msg::TransformStamped transform_stamped;
          try 
          {
              // transform_stamped = tf_buffer_.lookupTransform(target_frame, scan.header.frame_id, tf2::TimePointZero);
              transform_stamped = tf_buffer_.lookupTransform("base_scan",frame_id, rclcpp::Time(0));
 
          } 
          
          catch (tf2::TransformException &ex) 
          {
              RCLCPP_ERROR(this->get_logger(), "TF2 error: %s", ex.what());
              return;
          }
        }
        

        //LASER1 
        double angle_min_ = angle_min_deg_ * M_PI / 180.0;
        double angle_max_ = angle_max_deg_ * M_PI / 180.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) 
        {
            double angle_ = msg->angle_min + i * msg->angle_increment;
            if (angle_ >= angle_min_ && angle_ <= angle_max_) 
            {
                integradted_scan.ranges[i] = msg->ranges[i];
            }
        }

        scan_pub->publish(integradted_scan);
  }


private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_right;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_left;

    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub1;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub2;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub3;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub4;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
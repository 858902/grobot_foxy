// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "laser_geometry/laser_geometry.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_sensor_msgs/tf2_sensor_msgs.h"

// class LaserToPointcloudNode : public rclcpp::Node {
// public:
//     LaserToPointcloudNode() : Node("laser_to_pointcloud") {
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10,
//             std::bind(&LaserToPointcloudNode::scanCallback, this, std::placeholders::_1));
        
//         publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             "/pointcloud_lidar", 10);
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
//         sensor_msgs::msg::PointCloud2 cloud;


//         projector_.projectLaser(*scan, cloud);
//         sensor_msgs::msg::PointCloud2 cloud_transformed;
//         publisher_->publish(cloud_transformed);
    
//     }
    
//     laser_geometry::LaserProjection projector_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<LaserToPointcloudNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include <memory>

class LaserToPointcloudNode : public rclcpp::Node {
public:
    LaserToPointcloudNode() 
    : Node("laser_to_pointcloud"), 
      tf_buffer_(std::make_shared<rclcpp::Clock>(), tf2::durationFromSec(10.0)), // 여기를 수정했습니다.
      tf_listener_(tf_buffer_) {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserToPointcloudNode::scanCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_lidar", 10);
    }
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*scan, cloud, -1.0, laser_geometry::channel_option::Index);
        sensor_msgs::msg::PointCloud2 cloud_in_base_frame;
        std::string target_frame = "base_link";

        try {
            if (tf_buffer_.canTransform(target_frame, cloud.header.frame_id, cloud.header.stamp, tf2::durationFromSec(1.0))) {
                tf_buffer_.transform(cloud, cloud_in_base_frame, target_frame, tf2::durationFromSec(1.0)); // 1초 타임아웃 추가
            } else {
                RCLCPP_WARN(this->get_logger(), "No transform available.");
                return;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF2 exception: %s", ex.what());
            return;
        }

        publisher_->publish(cloud_in_base_frame);
    }

    laser_geometry::LaserProjection projector_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserToPointcloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

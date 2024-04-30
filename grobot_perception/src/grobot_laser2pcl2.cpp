#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

class LaserToPointcloudNode : public rclcpp::Node {
public:
    LaserToPointcloudNode() : Node("laser_to_pointcloud") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/LIDAR2/scan", 10,
            std::bind(&LaserToPointcloudNode::scanCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan/pointcloud", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*scan, cloud);
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        publisher_->publish(cloud_transformed);
    
    }
    
    laser_geometry::LaserProjection projector_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserToPointcloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

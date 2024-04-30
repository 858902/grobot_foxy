#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        // 변환 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // LIDAR와 카메라 토픽 구독
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/LIDAR2/scan", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::lidarCallback, this, std::placeholders::_1));

        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::cameraCallback, this, std::placeholders::_1));

        // 융합된 PointCloud2 데이터를 위한 퍼블리셔
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_cloud", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // LaserScan에서 PointCloud2로 변환
        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*scan_msg, cloud);

        // PointCloud2를 base_link 프레임으로 변환
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        if (tf_buffer_->canTransform(cloud.header.frame_id, "base_link", tf2::TimePointZero)) {
            tf2::doTransform(cloud, cloud_transformed, tf_buffer_->lookupTransform("base_link", cloud.header.frame_id, tf2::TimePointZero));
        }
        
        // 변환된 클라우드를 저장
        last_lidar_cloud_ = cloud_transformed;
    }

    void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // PointCloud2를 base_link 프레임으로 변환
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        if (tf_buffer_->canTransform(cloud_msg->header.frame_id, "base_link", tf2::TimePointZero)) {
            tf2::doTransform(*cloud_msg, cloud_transformed, tf_buffer_->lookupTransform("base_link", cloud_msg->header.frame_id, tf2::TimePointZero));
        }

        // last_lidar_cloud_가 마지막으로 처리된 LiDAR 데이터를 포함하고 있는 경우
        if (!last_lidar_cloud_.data.empty()) {
            // LiDAR 클라우드와 카메라 클라우드 융합 (여기서는 단순 연결로 처리)
            sensor_msgs::msg::PointCloud2 fused_cloud;
            // 여기에 더 복잡한 융합 알고리즘 적용 가능
            
            // 두 포인트클라우드를 단순 연결합니다. 실제 프로젝트에서는 보다 세련된 방법을 고려해야 합니다.
            concatenatePointCloud(last_lidar_cloud_, cloud_transformed, fused_cloud);

            // 융합된 클라우드를 발행
            cloud_publisher_->publish(fused_cloud);
        }
    }

    void concatenatePointCloud(const sensor_msgs::msg::PointCloud2 &cloud1, const sensor_msgs::msg::PointCloud2 &cloud2, sensor_msgs::msg::PointCloud2 &fused_cloud) {
        // 첫 번째 클라우드를 fused_cloud로 복사
        fused_cloud = cloud1;

        // 두 번째 클라우드 데이터 붙이기
        size_t data_size = cloud1.data.size() + cloud2.data.size();
        fused_cloud.data.reserve(data_size);

        fused_cloud.row_step = cloud1.row_step + cloud2.row_step;
        fused_cloud.width = cloud1.width + cloud2.width;
        fused_cloud.data.insert(fused_cloud.data.end(), cloud2.data.begin(), cloud2.data.end());
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::PointCloud2 last_lidar_cloud_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

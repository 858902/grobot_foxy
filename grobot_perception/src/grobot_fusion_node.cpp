#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        // 변환 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            // "/LIDAR2/scan", rclcpp::SensorDataQoS(),
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::lidarCallback, this, std::placeholders::_1));

        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::cameraCallback, this, std::placeholders::_1));

        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fused_cloud", 10);
        lidar_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_cloud", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        try {
            sensor_msgs::msg::PointCloud2 temp_cloud;
            projector_.projectLaser(*scan_msg, temp_cloud, -1.0, laser_geometry::channel_option::Intensity);
            last_lidar_cloud_ = temp_cloud;
            RCLCPP_INFO(this->get_logger(), "LaserScan to PointCloud2 conversion complete.");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform laser scan to point cloud: %s", ex.what());
        }
        lidar_cloud_publisher_->publish(last_lidar_cloud_);
    }

    void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *camera_cloud);


        for (auto& point : camera_cloud->points) {
            point.z = 0.0f;
        }
        sensor_msgs::msg::PointCloud2 camera_cloud_msg;
        pcl::toROSMsg(*camera_cloud, camera_cloud_msg);

        sensor_msgs::msg::PointCloud2 fused_cloud;
        // concatenatePointCloud(last_lidar_cloud_, *cloud_msg, fused_cloud);
        concatenatePointCloud(last_lidar_cloud_, camera_cloud_msg, fused_cloud);

        sensor_msgs::msg::PointCloud2 output_cloud_msg;
        downsampleAndCluster(fused_cloud, output_cloud_msg);

        cloud_publisher_->publish(output_cloud_msg);
    }

    void concatenatePointCloud(const sensor_msgs::msg::PointCloud2 &cloud1, const sensor_msgs::msg::PointCloud2 &cloud2, sensor_msgs::msg::PointCloud2 &fused_cloud) {
        
        fused_cloud = cloud1;
        size_t data_size = cloud1.data.size() + cloud2.data.size();
        fused_cloud.data.reserve(data_size);
        fused_cloud.row_step = cloud1.row_step + cloud2.row_step;
        fused_cloud.width = cloud1.width + cloud2.width;
        fused_cloud.data.insert(fused_cloud.data.end(), cloud2.data.begin(), cloud2.data.end());
    }

    void downsampleAndCluster(const sensor_msgs::msg::PointCloud2 &fused_cloud_msg, sensor_msgs::msg::PointCloud2 &output_cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(fused_cloud_msg, *input_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(input_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.0, 0.5f); // X 축 경계 설정
        pass_x.filter(*x_filtered_cloud);

        // Passthrough 필터 적용 - Z 축
        pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(x_filtered_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-0.5f, 0.0); // Z 축 경계 설정
        pass_z.filter(*z_filtered_cloud);

        // Voxel Grid 다운샘플링
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(z_filtered_cloud);
        vg.setLeafSize(0.03f, 0.03f, 0.03f); // 모든 축에 대해 0.03f로 설정
        vg.filter(*downsampled_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(downsampled_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.05);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);  
        ec.setSearchMethod(tree);
        ec.setInputCloud(downsampled_cloud);
        ec.extract(cluster_indices);

        // 군집화 결과를 output_cloud_msg에 저장
        pcl::toROSMsg(*downsampled_cloud, output_cloud_msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_cloud_publisher_;

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
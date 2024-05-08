#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h> // 추가
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("grobot_sensor_fusion_node") {
        // lidar1_subscriber_ = this->create_subscription
        // <sensor_msgs::msg::PointCloud2>(
        //     "/LIDAR1/scan", 10,
        //     std::bind(&SensorFusionNode::lidar1Callback, this, std::placeholders::_1));

        // lidar2_subscriber_ = this->create_subscription
        // <sensor_msgs::msg::PointCloud2>(
        //     "/LIDAR2/scan", 10,
        //     std::bind(&SensorFusionNode::lidar2Callback, this, std::placeholders::_1));
        lidar2_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/LIDAR2/scan", 10, std::bind(&SensorFusionNode::lidar2Callback, this, std::placeholders::_1));

        camera_subscriber_ = this->create_subscription
        <sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&SensorFusionNode::cameraCallback, this, std::placeholders::_1));

        fused_publisher_ = this->create_publisher
        <sensor_msgs::msg::PointCloud2>(
            "fused_point_cloud", 10);

        // tf broadcaster 초기화
        tf_broadcaster_ = std::make_shared
        <tf2_ros::TransformBroadcaster>(this);
    }

private:
    // void lidar1Callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    //     preprocessAndPublish(lidar_cloud);
    //     geometry_msgs::msg::TransformStamped transformStamped;
    //     transformStamped.header.stamp = rclcpp::Clock().now();
    //     transformStamped.header.frame_id = "lidar_frame";
    //     transformStamped.child_frame_id = "base_link";
    //     transformStamped.transform.translation.x = 0.0;
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = 0.0;
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, 0);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();
    //     tf_broadcaster_->sendTransform(transformStamped);
    // }

    // void lidar2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
    //     preprocessAndPublish(lidar_cloud);

    //     geometry_msgs::msg::TransformStamped transformStamped;
    //     transformStamped.header.stamp = lidar_msg->header.stamp;  // 시간 동기화
    //     transformStamped.header.frame_id = "base_link";
    //     transformStamped.child_frame_id = "base_scan2";
        
    //     transformStamped.transform.translation.x = 0.0;
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = 0.0;

    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, 0);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     tf_broadcaster_->sendTransform(transformStamped);
    // }
    void lidar2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
        // laser_geometry::LaserProjection projector_;
        // sensor_msgs::msg::PointCloud2 cloud;
        // projector_.projectLaser(*scan_msg, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*lidar_msg, *lidar_cloud);
        
        preprocessAndPublish(lidar_cloud);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = lidar_msg->header.stamp; // 시간 동기화
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "base_scan2";
        
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }

    void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr camera_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*camera_msg, *camera_cloud);
        preprocessAndPublish(camera_cloud);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = camera_msg->header.stamp;
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "camera_link";
        
        // 센서의 실제 위치
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }
    void preprocessAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
        // Preprocessing steps (e.g., voxel grid downsampling, outlier removal, etc.)
        // You can add your preprocessing steps here

        // Example: Voxel grid downsampling
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(input_cloud);
        voxel_grid_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid_filter.filter(*filtered_cloud);

        // Example: ICP
        // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // Set parameters for ICP if needed
        // icp.setMaxCorrespondenceDistance(0.05);
        // icp.setTransformationEpsilon(1e-8);
        // icp.setEuclideanFitnessEpsilon(1);
        // icp.setMaximumIterations(50);
        // icp.setRANSACOutlierRejectionThreshold(0.01);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(filtered_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_cloud);
        ec.extract(cluster_indices);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cluster->points.push_back(filtered_cloud->points[*pit]);
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
        }

        // Publish preprocessed point cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        fused_publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription
    <sensor_msgs::msg::PointCloud2>::SharedPtr lidar1_subscriber_;
    
    rclcpp::Subscription
    <sensor_msgs::msg::PointCloud2>::SharedPtr lidar2_subscriber_;
    
    rclcpp::Subscription
    <sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
    
    rclcpp::Publisher
    <sensor_msgs::msg::PointCloud2>::SharedPtr fused_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
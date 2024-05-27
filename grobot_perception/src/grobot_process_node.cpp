// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"
// #include "pcl/filters/voxel_grid.h"
// #include "pcl/kdtree/kdtree.h"
// #include "pcl/segmentation/extract_clusters.h"
// #include "pcl/filters/passthrough.h"
// #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
// #include "tf2_ros/static_transform_broadcaster.h"

// class CameraProcessing : public rclcpp::Node {
// public:
//     CameraProcessing() : Node("grobot_process_node") {
//         tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//         transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         camera_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/camera/depth/color/points", rclcpp::SensorDataQoS(),
//             std::bind(&CameraProcessing::cameraCallback, this, std::placeholders::_1));

//         cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_cloud", 10);
//     }

// private:
//     void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
//         sensor_msgs::msg::PointCloud2 cloud_in_base_frame;
//         std::string target_frame = "base_link";

//         try {
//             if (tf_buffer_->canTransform(target_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp)) {
//                 tf_buffer_->transform(*cloud_msg, cloud_in_base_frame, target_frame, tf2::durationFromSec(1.0)); // 1초 타임아웃 추가
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "No transform available.");
//                 return;
//             }
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(this->get_logger(), "TF2 exception: %s", ex.what());
//             return;
//         }

//         pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(cloud_in_base_frame, *camera_cloud); // 여기서 변환된 클라우드 사용

//         for (auto& point : camera_cloud->points) {
//             point.z = 0.0f; 
//         }

//         sensor_msgs::msg::PointCloud2 camera_cloud_msg;
//         pcl::toROSMsg(*camera_cloud, camera_cloud_msg);

//         sensor_msgs::msg::PointCloud2 output_cloud_msg;
//         downsampleAndCluster(camera_cloud_msg, output_cloud_msg);

//         cloud_publisher_->publish(output_cloud_msg);
//     }


//     void downsampleAndCluster(const sensor_msgs::msg::PointCloud2 &input_cloud_msg, sensor_msgs::msg::PointCloud2 &output_cloud_msg) {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(input_cloud_msg, *input_cloud);


//         // ROI 설정 ---
//         pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PassThrough<pcl::PointXYZ> pass_x;
//         pass_x.setInputCloud(input_cloud);
//         pass_x.setFilterFieldName("x");
//         pass_x.setFilterLimits(0.0, 5.0);
//         pass_x.filter(*x_filtered_cloud);

//         pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PassThrough<pcl::PointXYZ> pass_z;
//         pass_z.setInputCloud(x_filtered_cloud);
//         pass_z.setFilterFieldName("z");
//         pass_z.setFilterLimits(-0.5f, 0.5f);
//         pass_z.filter(*z_filtered_cloud);

//         // ROI -------

//         // VoxelGrid DownSampling ------
//         pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::VoxelGrid<pcl::PointXYZ> vg;
//         vg.setInputCloud(z_filtered_cloud);
//         vg.setLeafSize(0.03f, 0.03f, 0.03f); // 모든 축에 대해 0.03f로 설정
//         vg.filter(*downsampled_cloud);
//         // VG ---------------------------
        

//         // Clustering ----------------
//         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//         tree->setInputCloud(downsampled_cloud);
//         std::vector<pcl::PointIndices> cluster_indices;
//         pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//         ec.setClusterTolerance(0.05);
//         ec.setMinClusterSize(50);
//         ec.setMaxClusterSize(25000);
//         ec.setSearchMethod(tree);
//         ec.setInputCloud(downsampled_cloud);
//         ec.extract(cluster_indices);
//         // Clustering ----------------

//         pcl::toROSMsg(*downsampled_cloud, output_cloud_msg);
//     }

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraProcessing>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/passthrough.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/static_transform_broadcaster.h"

class CameraProcessing : public rclcpp::Node {
public:
    CameraProcessing() : Node("grobot_process_node") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&CameraProcessing::cameraCallback, this, std::placeholders::_1));

        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_cloud", 10);
    }

private:
    void cameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        sensor_msgs::msg::PointCloud2 cloud_in_base_frame;
        std::string target_frame = "base_link";

        try {
            if (tf_buffer_->canTransform(target_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp)) {
                tf_buffer_->transform(*cloud_msg, cloud_in_base_frame, target_frame, tf2::durationFromSec(1.0)); // 1초 타임아웃 추가
            } else {
                RCLCPP_WARN(this->get_logger(), "No transform available.");
                return;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF2 exception: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_in_base_frame, *camera_cloud); // 여기서 변환된 클라우드 사용

        sensor_msgs::msg::PointCloud2 camera_cloud_msg;
        pcl::toROSMsg(*camera_cloud, camera_cloud_msg);

        sensor_msgs::msg::PointCloud2 output_cloud_msg;
        downsampleAndCluster(camera_cloud_msg, output_cloud_msg);

        cloud_publisher_->publish(output_cloud_msg);
    }

    void downsampleAndCluster(const sensor_msgs::msg::PointCloud2 &input_cloud_msg, sensor_msgs::msg::PointCloud2 &output_cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(input_cloud_msg, *input_cloud);

        // ROI 설정 ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(input_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.0, 5.0);
        pass_x.filter(*x_filtered_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(x_filtered_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-0.5f, 0.5f);
        pass_z.filter(*z_filtered_cloud);

        // VoxelGrid DownSampling ------
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(z_filtered_cloud); // ROI 필터링된 클라우드를 사용
        vg.setLeafSize(0.03f, 0.03f, 0.03f); // 모든 축에 대해 0.03f로 설정
        vg.filter(*downsampled_cloud);

        RCLCPP_INFO(this->get_logger(), "Downsampled cloud size: %zu", downsampled_cloud->points.size());

        // Clustering ----------------
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

        RCLCPP_INFO(this->get_logger(), "Number of clusters: %zu", cluster_indices.size());

        // z축 기준 밀도 판단과 xy 평면 프로젝션
        float z_threshold = 0.1; // 임의의 z축 임계값 설정
        pcl::PointCloud<pcl::PointXYZ>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& index : indices.indices) {
                cluster->points.push_back(downsampled_cloud->points[index]);
            }
            if (is_dense(cluster, z_threshold)) {
                *dense_cloud += *cluster;
            } else {
                *sparse_cloud += *cluster;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Dense cloud size: %zu, Sparse cloud size: %zu", dense_cloud->points.size(), sparse_cloud->points.size());

        // xy 평면으로 프로젝션
        pcl::PointCloud<pcl::PointXYZ>::Ptr dense_projected = project_to_xy(dense_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_projected = project_to_xy(sparse_cloud);

        // 결과 병합
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *final_cloud += *dense_projected;
        *final_cloud += *sparse_projected;

        RCLCPP_INFO(this->get_logger(), "Final cloud size after projection: %zu", final_cloud->points.size());

        pcl::toROSMsg(*final_cloud, output_cloud_msg);
        output_cloud_msg.header = input_cloud_msg.header; // 헤더 설정
    }

    bool is_dense(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float threshold) {
        for (const auto& point : cloud->points) {
            if (point.z > threshold) {
                return true;
            }
        }
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr project_to_xy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            pcl::PointXYZ projected_point;
            projected_point.x = point.x;
            projected_point.y = point.y;
            projected_point.z = 0.0;
            projected_cloud->points.push_back(projected_point);
        }
        return projected_cloud;
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

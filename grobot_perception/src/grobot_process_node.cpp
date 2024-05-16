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
        
        // sensor_msgs::msg::PointCloud2 cloud_in_base_frame;
        // std::string target_frame = "base_link";

        // try {
        //     if (tf_buffer_->canTransform(target_frame, cloud_msg->header.frame_id, tf2::TimePointZero)) {
        //         tf_buffer_->transform(*cloud_msg, cloud_in_base_frame, target_frame);
        //     } else {
        //         RCLCPP_WARN(this->get_logger(), "No transform available.");
        //         return;
        //     }
        // } catch (tf2::TransformException &ex) {
        //     RCLCPP_WARN(this->get_logger(), "TF2 exception: %s", ex.what());
        //     return;
        // }
        // 수정된 코드:
        std::string target_frame = "base_link"; // 변환하려는 대상 frame ID
        tf2::Duration transform_timeout(std::chrono::seconds(1)); // 변환 시도 최대 대기 시간 설정

        // cloud_msg의 timestamp를 기반으로 tf2::TimePoint 객체 생성
        tf2::TimePoint time_point = tf2::TimePoint(std::chrono::milliseconds(cloud_msg->header.stamp.sec * 1000LL + cloud_msg->header.stamp.nanosec / 1000000LL));

        // 변환된 포인트 클라우드를 저장할 객체
        sensor_msgs::msg::PointCloud2 cloud_in_base_frame;

        try {
            // transform 함수를 올바르게 호출
            tf_buffer_->transform(*cloud_msg, cloud_in_base_frame, target_frame, time_point, target_frame, transform_timeout);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform cloud: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_in_base_frame, *camera_cloud);

        for (auto& point : camera_cloud->points) {
            point.z = 0.0f; 
        }

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
        pass_x.setFilterLimits(0.0, 0.5f);
        pass_x.filter(*x_filtered_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(x_filtered_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-0.5f, 0.0);
        pass_z.filter(*z_filtered_cloud);

        // ROI -------

        // VoxelGrid DownSampling ------
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(z_filtered_cloud);
        vg.setLeafSize(0.03f, 0.03f, 0.03f); // 모든 축에 대해 0.03f로 설정
        vg.filter(*downsampled_cloud);
        // VG ---------------------------
        

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
        // Clustering ----------------

        pcl::toROSMsg(*downsampled_cloud, output_cloud_msg);
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

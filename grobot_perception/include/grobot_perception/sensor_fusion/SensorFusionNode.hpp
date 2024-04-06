#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#include <yaml-cpp/yaml.h>

#include <pcl_conversions/pcl_conversions.h>
// TODO delete this
#include <pcl/io/pcd_io.h>

#include "zed_msgs/msg/bounding_box_list2d.hpp"
#include "zed_msgs/msg/bounding_box2d.hpp"
#include "zed_msgs/msg/cone.hpp"
#include "zed_msgs/msg/cones.hpp"

#include "cv_bridge/cv_bridge.h"

#include "SensorFusionUtils.hpp"

// using std::placeholders::_1;

class SensorFusionNode : public rclcpp::Node{
    CameraConfig cameraConfig_;
    LidarConfig lidarConfig_;
    Translation lidar2CameraTranslation_;

    // Names of the topics to subscribe to

    // Cones point cloud
    std::string pointcloud_topic_name_;
    // BB list
    std::string bboxes_topic_name_;
    // Zed left camera image, used for visual debug
    std::string image_topic_name_;
    // Output cones
    std::string cones_topic_name_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Subscription<zed_msgs::msg::BoundingBoxList2d>::SharedPtr bounding_boxes_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr zed_image_subscriber;

    // Publisher
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr fusion_cones_publisher;

    pcl::PointCloud<pcl::PointXYZI> point_cloud_;

    // Create a queue of elements, so I can match the closest bb-lidar_scan given timestamps
    BoundingBoxesQueue bboxes_queue_;

    // 3D Points only, used to store the point cloud's point without all the rest of the information(height, width, intensity, header, etc)
    std::vector<cv::Point3f> pointcloud_points_;

    // Projected points to work with
    std::vector<cv::Point2f> projected_points_;

    // Image for visual debug
    cv_bridge::CvImagePtr zed_image_;

    // Flag used to show(or not show) the visual debug window
    bool visual_debug_on_ = false;

    // coefficients for fix bbounding boxes size estimation
    float x_factor_ = 1.0;
    float y_factor_ = 1.0;

    // Result cones message(has 5 list each and every of the 4 cones type + 1 for eventually blank cones)
    zed_msgs::msg::Cones resultCones_;

    /**
     * @brief setup calibration parametrs reading eventual parameter from .yaml file(s)
     */
    void configCalibration();

    /**
     * @brief setup node's publishers and subscriptions, reading parameters from .yaml file(s)
     */
    void configTopics();

    // Main function
    void computeFusionCones();

    /**
     * @brief fuse information beyween a single bounding box and a point in space.
     *
     * @param bbox bounding box
     * @param point point to get spacial informations from
     */
    void createFuseCone(zed_msgs::msg::BoundingBox2d& bbox, cv::Point3f& point);

    /**
     * @brief Setter for parameter point_cloud_, called each time the subscription's callback is called
     */
    void setPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud);

    /**
     * @brief Setter for parameter bounding_boxes, called each time the subscription's callback is called
     */
    void setBoundingBoxes(const zed_msgs::msg::BoundingBoxList2d::SharedPtr bounding_boxes);

    /**
     * @brief Setter for the image for visual debug
     */
    void setZedImage(const sensor_msgs::msg::Image::SharedPtr image);

    /**
     * @brief Print the projected points and bounding boxes in a window for visual debug
     */
    void visualDebug() const;

    /**
     * @brief Extract PointCloud data in cv::Point3f data, for a more easly use
     */
    void extractPointsFromPointCloud() noexcept;

    /**
     * @brief Using OpenCv, intrinsic and extrinsec camera parameters, project the 3d point cloud points to a 2d plane, "the lidar image"
     *
     * @param pointCloudIn vector of 3d point rappresenting the point cloud 3d point
     * * @param pointCloudOut vector of 2d point rappresenting the 2d projection of the input points
     */
    inline void project3DPointsTo2DPlane(const std::vector<cv::Point3f> &pointCloudIn, std::vector<cv::Point2f> &planePointsOut);

    /**
     * @brief Given the fact that the origin of the 2d point is the center of "the lidar image" i have to transform the points to the new origin
     *
     * @param planePoints points to transform
     * @param image_w width dimension of "the lidar image"
     * @param image_h height dimension of "the lidar image"
     */
    void transform2DPointsOrigin(std::vector<cv::Point2f> &planePoints, const float image_w, const float image_h);

    /**
     * @brief Scale "the lidar image" to the camera image, used so i can overlap the BBs and the resultant points
     *
     * @param planePoints points to scale
     * @param inputImageW input image original W
     * @param inputImageH input image original H
     * @param outputImageW output image original W
     * @param outputImageW output image original H
     */
    void scale2DPointsToImage(std::vector<cv::Point2f> &planePoints, const float inputImageW, const float inputImageH, const float outputImageW,
                              const float outputImageH);

    /**
     * @brief Transform the normalized bb's coordinates into specific image coordinates
     *
     * @param image_dimensions a pair containing width and height of the image(respectivly first and second element)
     */
    /* void denormalizeBoundingBoxes(const std::pair<int, int> image_dimensions) noexcept; */

    // TODO cambiare nome a questa funzione, crè "fuse"???
    /**
     * @brief
     */
    void elaborateCones();

    /**
     * @brief compute euclideanDistance
     * @param first_point first pair containing x and y of the first point(respectivly first and second element)
     * @param second_point second pair containing x and y of the second point(respectivly first and second element)
     */
    inline float euclideanDistance(const std::pair<float, float> first_point, const std::pair<float, float> second_point) const noexcept;

public:
    SensorFusionNode();

    const int BIG_ORANGE_CONE = 1;
    const int BLUE_CONE = 2;
    const int LITTLE_ORANGE_CONE = 3;
    const int YELLOW_CONE = 4;
};
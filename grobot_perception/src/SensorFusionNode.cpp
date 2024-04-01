#include "SensorFusionNode.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

SensorFusionNode::SensorFusionNode() : Node("SensorFusionNode")
{
    this->configCalibration();
    this->configTopics();
}

void SensorFusionNode::configTopics()
{
    YAML::Node nodeSettingsFile;

    this->declare_parameter("node_yaml_config_path");
    rclcpp::Parameter node_config_file = this->get_parameter("node_yaml_config_path");

    try
    {
        nodeSettingsFile = YAML::LoadFile(node_config_file.as_string());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Errore nell'apertura del file di configurazione del nodo.");
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    this->pointcloud_topic_name_ = nodeSettingsFile["input_lidar_topic"].as<std::string>();
    this->bboxes_topic_name_ = nodeSettingsFile["input_bounding_boxes_topic"].as<std::string>();
    this->image_topic_name_ = nodeSettingsFile["input_camera_topic"].as<std::string>();
    this->cones_topic_name_ = nodeSettingsFile["output_topic"].as<std::string>();
    this->visual_debug_on_ = nodeSettingsFile["visual_debug"].as<bool>();

    RCLCPP_INFO(this->get_logger(), this->pointcloud_topic_name_);
    RCLCPP_INFO(this->get_logger(), this->bboxes_topic_name_);
    RCLCPP_INFO(this->get_logger(), this->image_topic_name_);

    if (this->visual_debug_on_)
    {
        RCLCPP_INFO(this->get_logger(), "Visual debug is ON!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Visual debug is OFF!");
    }

    this->point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->pointcloud_topic_name_, 10, [this](sensor_msgs::msg::PointCloud2::SharedPtr pointCloud)
                                                                                               { this->setPointCloud(pointCloud); });

    this->bounding_boxes_subscription_ = this->create_subscription<zed_msgs::msg::BoundingBoxList2d>(this->bboxes_topic_name_, 10, [this](zed_msgs::msg::BoundingBoxList2d::SharedPtr boundingBoxes)
                                                                                                     { this->setBoundingBoxes(boundingBoxes); });

    this->zed_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(this->image_topic_name_, 10, [this](sensor_msgs::msg::Image::SharedPtr image)
                                                                                    { this->setZedImage(image); });

    this->fusion_cones_publisher = this->create_publisher<zed_msgs::msg::Cones>(this->cones_topic_name_, 1);
}

void SensorFusionNode::configCalibration()
{
    /*TRANSLATION CONFIGURATION*/
    this->declare_parameter("calibration_yaml_config_path");
    rclcpp::Parameter calibrationSettingsFile = this->get_parameter("calibration_yaml_config_path");

    YAML::Node calibrationSettingsNode;

    try
    {
        calibrationSettingsNode = YAML::LoadFile(calibrationSettingsFile.as_string());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Errore nell'apertura del file di configurazione di calibrazione.");
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    this->lidar2CameraTranslation_.x = calibrationSettingsNode["roto_translation"]["translation"]["x"].as<float>();
    this->lidar2CameraTranslation_.y = calibrationSettingsNode["roto_translation"]["translation"]["y"].as<float>();
    this->lidar2CameraTranslation_.z = calibrationSettingsNode["roto_translation"]["translation"]["z"].as<float>();

    this->x_factor_ = calibrationSettingsNode["bbox_estimate"]["x_factor"].as<float>();
    this->y_factor_ = calibrationSettingsNode["bbox_estimate"]["y_factor"].as<float>();

    /* CAMERA CONFIGURATION */

    this->declare_parameter("camera_yaml_config_path");
    rclcpp::Parameter camera_config_file = this->get_parameter("camera_yaml_config_path");

    YAML::Node cameraSettingsFile;
    YAML::Node lidarSettingsFile;

    try
    {
        cameraSettingsFile = YAML::LoadFile(camera_config_file.as_string());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Errore nell'apertura del file di configurazione della camera.");
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    YAML::Node cameraConfigNode = cameraSettingsFile["Camera_config"];

    this->cameraConfig_.projection_matrix = cv::Mat(3, 3, cv::DataType<float>::type);
    // this->cameraConfig_.projection_matrix = cv::Mat(3, 4, cv::DataType<float>::type);

    // First row
    this->cameraConfig_.projection_matrix.at<float>(0, 0) = cameraConfigNode["focal_lenghts"]["x"].as<float>();
    this->cameraConfig_.projection_matrix.at<float>(0, 1) = 0;
    this->cameraConfig_.projection_matrix.at<float>(0, 2) = cameraConfigNode["optical_centers"]["x"].as<float>();
    // this->cameraConfig_.projection_matrix.at<float>(0, 3) = 0;

    // Second row
    this->cameraConfig_.projection_matrix.at<float>(1, 0) = 0;
    this->cameraConfig_.projection_matrix.at<float>(1, 1) = cameraConfigNode["focal_lenghts"]["y"].as<float>();
    this->cameraConfig_.projection_matrix.at<float>(1, 2) = cameraConfigNode["optical_centers"]["y"].as<float>();
    // this->cameraConfig_.projection_matrix.at<float>(1, 3) = 0;

    // Third row
    this->cameraConfig_.projection_matrix.at<float>(2, 0) = 0;
    this->cameraConfig_.projection_matrix.at<float>(2, 1) = 0;
    this->cameraConfig_.projection_matrix.at<float>(2, 2) = 1;
    // this->cameraConfig_.projection_matrix.at<float>(2, 3) = 0;

    std::cout << this->cameraConfig_.projection_matrix << std::endl;

    this->cameraConfig_.distortion_coefficients = cv::Mat(4, 1, cv::DataType<float>::type);

    this->cameraConfig_.distortion_coefficients.at<float>(0) = cameraConfigNode["distorsion_coefficients"]["k1"].as<float>();
    this->cameraConfig_.distortion_coefficients.at<float>(1) = cameraConfigNode["distorsion_coefficients"]["k2"].as<float>();
    this->cameraConfig_.distortion_coefficients.at<float>(2) = cameraConfigNode["distorsion_coefficients"]["p1"].as<float>();
    this->cameraConfig_.distortion_coefficients.at<float>(3) = cameraConfigNode["distorsion_coefficients"]["p2"].as<float>();

    cv::Mat rotation_vector = cv::Mat(3, 1, cv::DataType<float>::type);
    rotation_vector.convertTo(rotation_vector, CV_64F);
    rotation_vector.at<float>(0) = cameraConfigNode["rotation_vector"]["x"].as<float>();
    rotation_vector.at<float>(1) = cameraConfigNode["rotation_vector"]["y"].as<float>();
    rotation_vector.at<float>(2) = cameraConfigNode["rotation_vector"]["z"].as<float>();
    cv::Rodrigues(rotation_vector, this->cameraConfig_.rodriguez_rotation_vector);

    this->cameraConfig_.translation_vector = cv::Mat(3, 1, cv::DataType<float>::type);
    this->cameraConfig_.translation_vector.at<float>() = cameraConfigNode["translation_vector"]["x"].as<float>();
    this->cameraConfig_.translation_vector.at<float>() = cameraConfigNode["translation_vector"]["y"].as<float>();
    this->cameraConfig_.translation_vector.at<float>() = cameraConfigNode["translation_vector"]["z"].as<float>();

    this->cameraConfig_.camera_width = cameraConfigNode["camera_measures"]["camera_width"].as<int>();
    this->cameraConfig_.camera_height = cameraConfigNode["camera_measures"]["camera_height"].as<int>();

    /* LIDAR CONFIGURATION */

    this->declare_parameter("lidar_yaml_config_path");
    rclcpp::Parameter lidar_config_file = this->get_parameter("lidar_yaml_config_path");

    try
    {
        lidarSettingsFile = YAML::LoadFile(lidar_config_file.as_string());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Errore nell'apertura del file di configurazione del Lidar.");
        RCLCPP_WARN(this->get_logger(), e.what());
    }

    YAML::Node lidarConfigNode = lidarSettingsFile["Lidar_config"];

    this->lidarConfig_.lidar_HFOV = lidarConfigNode["FOVS"]["horizontal"].as<float>();
    this->lidarConfig_.lidar_VFOV = lidarConfigNode["FOVS"]["vertical"].as<float>();

    this->lidarConfig_.horizontal_resolution = lidarConfigNode["resolutions"]["horizontal"].as<float>();
    this->lidarConfig_.vertical_resolution = lidarConfigNode["resolutions"]["vertical"].as<float>();

    this->lidarConfig_.lidarImageW = this->lidarConfig_.lidar_HFOV / this->lidarConfig_.horizontal_resolution;
    this->lidarConfig_.lidarImageH = this->lidarConfig_.lidar_VFOV / this->lidarConfig_.vertical_resolution;
}

void SensorFusionNode::setPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud)
{
    pcl::fromROSMsg(*pointCloud, this->point_cloud_);

    // Scambio i campi del point cloud in modo da cambiare il sistema di riferimento. Nessuna trasformazione, nessuna rotazione, scambio solamente
    // gli assi del sistema di refiremento
    pcl::PointCloud<pcl::PointXYZI> new_point_cloud;
    new_point_cloud.header = this->point_cloud_.header;
    auto points = this->point_cloud_.points;

    float tmp_x = 0;

    for (pcl::PointXYZI point : points)
    {

        // Traslo agricolmente i punti nel sistema di riferimento della zed(credo)
        point.x += this->lidar2CameraTranslation_.x;
        point.y += this->lidar2CameraTranslation_.y;
        point.z += this->lidar2CameraTranslation_.z;

        tmp_x = point.x;
        pcl::PointXYZI new_point = pcl::PointXYZI();
        new_point.x = -point.y;
        new_point.y = -point.z;
        new_point.z = tmp_x;
        new_point.intensity = point.intensity;
        new_point_cloud.push_back(new_point);
    }

    this->point_cloud_.swap(new_point_cloud);

    this->computeFusionCones();
}

void SensorFusionNode::setBoundingBoxes(const zed_msgs::msg::BoundingBoxList2d::SharedPtr boundingBoxes)
{
    this->bboxes_queue_.push(*boundingBoxes);
}

void SensorFusionNode::setZedImage(const sensor_msgs::msg::Image::SharedPtr image)
{
    /* RCLCPP_INFO(this->get_logger(), "Sto deserializzando l'immagine della Zed"); */

    this->zed_image_ = cv_bridge::toCvCopy(image, image->encoding);
}

void SensorFusionNode::visualDebug() const
{
    // If the zed has sent at least one full image, I can draw. If the Zed has never sent anything, this->zed_image_ is null, thus an easy seg fault
    if (this->zed_image_ != nullptr)
    {

        try
        {

            BoundingBoxesQueue bounding_boxes_tmp = this->bboxes_queue_;

            // Take the bounding box list with smallest delta time(difference in time between the bounding box capture and lidar scan)
            zed_msgs::msg::BoundingBoxList2d closest_bb = bounding_boxes_tmp.closestBoundingBox(this->point_cloud_.header.stamp);

            // Disegna i box sull'immagine
            for (auto cone : closest_bb.list_of_boxes)
            {

                float resized_x = cone.x * this->cameraConfig_.camera_width / 416;
                float resized_y = cone.y * this->cameraConfig_.camera_height / 416;
                float resized_w = cone.width * this->cameraConfig_.camera_width / 416;
                float resized_h = cone.height * this->cameraConfig_.camera_height / 416;

                cv::Rect r = cv::Rect(resized_x - (resized_w / 2), resized_y - (resized_h / 2), resized_w, resized_h * 2);
                cv::rectangle(zed_image_->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(zed_image_->image, std::to_string(cone.cone_type), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0xFF, 0xFF, 0xFF), 2);

                // cv::putText(zed_image_->image, std::to_string(cone.), cv::Point(r.x + 20, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xA), 2);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "C'è stato un problema nel disegnare i Bounding Boxes");
            RCLCPP_WARN(this->get_logger(), e.what());
        }

        try
        {
            // disegna i punti proiettati sull'immagine
            for (auto point : this->projected_points_)
            {
                RCLCPP_INFO(this->get_logger(), "Sto proiettando il punto con coords: %f - %f", point.x, point.y);
                //                                                R  G  R
                cv::circle(zed_image_->image, point, 5, cv::Scalar(0, 0, 255), -1);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "C'è stato un problema nel disegnare i punti proiettati");
            RCLCPP_WARN(this->get_logger(), e.what());
        }

        try
        {
            cv::imshow("Visual Debug", zed_image_->image);
            cv::waitKey(1);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "C'è stato un problema nella creazione/dispaly della finestra.");
            RCLCPP_WARN(this->get_logger(), e.what());
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "La zed non sta ancora pubblicando /zed_images");
    }
}

void SensorFusionNode::extractPointsFromPointCloud() noexcept
{
    // Clear the vector, so I don't have old data in it in the new iteration of the fusion
    this->pointcloud_points_.clear();

    for (long unsigned int i = 0; i < this->point_cloud_.points.size(); i++)
    {
        // effettuo anche la traslazione dei punti //TODO DECIDERE SE farlo in due operazioni separate
        cv::Point3f point = cv::Point3f(this->point_cloud_.points[i].x,
                                        this->point_cloud_.points[i].y,
                                        this->point_cloud_.points[i].z);

        this->pointcloud_points_.push_back(point);
    }
}

/*
void SensorFusionNode::denormalizeBoundingBoxes(const std::pair<int, int> image_dimensions) noexcept
{

    for (zed_msg::msg::BoundingBox3d &bb : this->bboxes_queue_.list_of_boxes)
    {
        bb.box[0] = bb.box[0] * image_dimensions.first;
        bb.box[1] = bb.box[1] * image_dimensions.second;
        bb.box[2] = bb.box[2] * image_dimensions.first;
        bb.box[3] = bb.box[3] * image_dimensions.second;

        std::cout << "bb x:" << bb.box[0]
                  << " - y: " << bb.box[1]
                  << " - w: " << bb.box[2]
                  << " - h: " << bb.box[3]
                  << std::endl;
    }
}
 */

void SensorFusionNode::computeFusionCones()
{

    this->projected_points_.clear();
    this->resultCones_.yellow_cones.clear();
    this->resultCones_.blue_cones.clear();
    this->resultCones_.little_orange_cones.clear();
    this->resultCones_.big_orange_cones.clear();
    // this->resultCones_.blank_cones.clear();

    auto startTime = std::chrono::steady_clock::now();

    this->extractPointsFromPointCloud();

    auto beforeProjectionTime = std::chrono::steady_clock::now();

    this->project3DPointsTo2DPlane(this->pointcloud_points_, this->projected_points_);

    auto afterProjectionTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(afterProjectionTime - beforeProjectionTime);

    RCLCPP_INFO(this->get_logger(), "Tempo per effettuare la proiezione %d ms", ellapsedTime.count());

    this->elaborateCones();

    auto afterMatchingTime = std::chrono::steady_clock::now();
    ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(afterMatchingTime - afterProjectionTime);
    RCLCPP_INFO(this->get_logger(), "Tempo per effettuare la proiezione %d ms", ellapsedTime.count());

    if (this->visual_debug_on_)
    {
        this->visualDebug();
    }

    // RCLCPP_INFO(this->get_logger(), "Numero coni blue: %d", this->resultCones_.blue_cones.size());
    // RCLCPP_INFO(this->get_logger(), "Numero coni gialli: %d", this->resultCones_.yellow_cones.size());
    // RCLCPP_INFO(this->get_logger(), "Numero coni arancioni grandi: %d", this->resultCones_.big_orange_cones.size());
    // RCLCPP_INFO(this->get_logger(), "Numero coni arancioni piccoli: %d", this->resultCones_.little_orange_cones.size());

    this->fusion_cones_publisher->publish(this->resultCones_);

    return;
}

void SensorFusionNode::project3DPointsTo2DPlane(const std::vector<cv::Point3f> &pointCloudIn, std::vector<cv::Point2f> &planePointsOut)
{

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);

    try
    {
        cv::projectPoints(pointCloudIn, R, t,
                          this->cameraConfig_.projection_matrix, cv::Mat(), planePointsOut);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

// TODO controllare se effettivamente questa funzione serve
void SensorFusionNode::transform2DPointsOrigin(std::vector<cv::Point2f> &planePoints, const float image_w, const float image_h)
{

    // Translate the point from the actual origin(dead center of the "image") to the standard center of an image(top left corner)
    for (cv::Point2f &point : planePoints)
    {
        // CAREFULL HERE: i use image_w&image_h / 2 because the non divided measure are the "full dimension" of the image. Given the fact
        // that the origin of the points is the middle of the image, I divide by two the dimensions

        point.x = (image_w / 2) + point.x;
        point.y = (image_h / 2) - point.y;

        // Inverto x e y, perchè devo invertire gli assi. Nella proiezione dei punti, abbiamo un sistema con x sulle ordinate, ed y sulle ascisse,
        // mentre nell'immagine risultante abbiamo un sistema con x sulle ascisse e y sulle ordinate.
        // tmp_point_x = point.x;
        // point.x = (image_w / 2) + point.y;
        // point.y = (image_h / 2) - tmp_point_x;
    }
}

void SensorFusionNode::scale2DPointsToImage(std::vector<cv::Point2f> &planePoints, const float inputImageW, const float inputImageH,
                                            const float outputImageW, const float outputImageH)
{
    for (cv::Point2f &point : planePoints)
    {
        point.x = (point.x * outputImageW) / inputImageW;
        point.y = (point.y * outputImageH) / inputImageH;
    }
}

void SensorFusionNode::createFuseCone(zed_msgs::msg::BoundingBox2d &bbox, cv::Point3f &point)
{
    zed_msgs::msg::Cone cone;
    cone.x = point.x,
    cone.y = point.y;
    cone.z = point.z;

    // Zed tracking
    cone.tracking_id = bbox.tracking_id;
    cone.cone_type = bbox.cone_type;

    switch (cone.cone_type)
    {
    case CONE_TYPE::YELLOW:
        this->resultCones_.yellow_cones.push_back(cone);
        break;
    case CONE_TYPE::BLUE:
        this->resultCones_.blue_cones.push_back(cone);
        break;
    case CONE_TYPE::LITTLE_ORANGE:
        this->resultCones_.little_orange_cones.push_back(cone);
        break;
    case CONE_TYPE::BIG_ORANGE:
        this->resultCones_.big_orange_cones.push_back(cone);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Colore del cono non riconosciuto durante il matching");
        break;
    }
}

void SensorFusionNode::elaborateCones()
{

    float closest_distance_from_bbox = 0;
    float centroid_distance_from_bbox = 0;

    // Take the bounding box list with smallest delta time(difference in time between the bounding box capture and lidar scan)

    zed_msgs::msg::BoundingBoxList2d bboxes_list = this->bboxes_queue_.closestBoundingBox(this->point_cloud_.header.stamp);

    for (zed_msgs::msg::BoundingBox2d bbox : bboxes_list.list_of_boxes)
    {
        // Every time I analyze a new box, set the min_distance to a huge value, like a "reset"
        closest_distance_from_bbox = 100000;

        // com,pute half dimensions of bounding box, then multiply for a factor
        float half_ww = bbox.width / 2.0 * x_factor_;
        float half_hh = bbox.height / 2.0 * y_factor_;

        // index of closest centroid, if it remeains negative no valid centroid has been found
        int closest_centroid_index = -1;

        for (std::size_t i = 0; i < this->projected_points_.size(); i++)
        {
            cv::Point2f point = projected_points_[i];
            // discard all points from the extented bbox
            if ((point.x < bbox.x - half_ww) || (point.x > bbox.x + half_ww))
                continue;
            if ((point.y < bbox.y - half_hh) || (point.y > bbox.y + half_hh))
                continue;
            // Compute the distance between the center of the BB and the projected 2d point
            centroid_distance_from_bbox = this->euclideanDistance(std::make_pair(bbox.x, bbox.y),
                                                                  std::make_pair(this->projected_points_[i].x, this->projected_points_[i].y));
            // If the new point is closer to the bb's center than the older update the closest centroid index variable

            if (centroid_distance_from_bbox < closest_distance_from_bbox)
            {
                closest_centroid_index = i;
                closest_distance_from_bbox = centroid_distance_from_bbox;
            }
        }

        if (closest_centroid_index > -1)
        { // A valid centroid has been found. IT'S FUSION TIME!
            this->createFuseCone(bbox, pointcloud_points_[closest_centroid_index]);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "bbox non associabile a nessun centroide!");
            // TODO: prendere cono 3d dalla detection della ZED.
        }
    }
}

inline float SensorFusionNode::euclideanDistance(const std::pair<float, float> first_point, const std::pair<float, float> second_point) const noexcept
{
    return sqrt(pow(second_point.first - first_point.first, 2) + (pow(second_point.second - first_point.second, 2)));
}
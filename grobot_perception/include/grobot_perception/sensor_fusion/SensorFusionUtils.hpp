#include <queue>
#include "zed_msgs/msg/cones.hpp"

typedef struct{
    cv::Mat rodriguez_rotation_vector;
    cv::Mat translation_vector;

    cv::Mat projection_matrix;
    cv::Mat distortion_coefficients;

    int camera_width;
    int camera_height;
} CameraConfig;

typedef struct{
    float lidar_HFOV;
    float lidar_VFOV;
    float horizontal_resolution;
    float vertical_resolution;

    int lidarImageW;
    int lidarImageH;

    // TODO aggiungere altri parametri(?)

} LidarConfig;

typedef struct
{
    float x;
    float y;
    float z;
} Translation;

enum CONE_TYPE {YELLOW, BLUE, LITTLE_ORANGE, BIG_ORANGE};

class BoundingBoxesQueue{
public:
    BoundingBoxesQueue(): max_queue_size_(10){}

    BoundingBoxesQueue(size_t queue_size): max_queue_size_(queue_size){}

    void push(zed_msgs::msg::BoundingBoxList2d new_element){
        // Pop the oldest element when the maximun queue size is reached
        if (queue_.size() >= max_queue_size_){
            queue_.pop();
        }

        // Add the element and increase size
        queue_.push(new_element);
    }

    void pop(){
        if (!queue_.empty())
            queue_.pop();
    }

    std::size_t size() const{
        return queue_.size();
    }

    zed_msgs::msg::BoundingBoxList2d closestBoundingBox(std::uint64_t lidar_timestamp){
        // Take the oldest element, then pop it from the queue
        zed_msgs::msg::BoundingBoxList2d candidate = queue_.front();
        this->pop();

        // Take the second oldest element
        zed_msgs::msg::BoundingBoxList2d other_candidate = queue_.front();

        // If the oldest element is further from the lidar scan than the second oldest, take another step in the queue, confronting the now oldest
        // with the third oldest(now second), until I find the one closest to the lidar timestamp. We exploit the fact that timestamps are not
        // decremental, so I can stop searching forward after a timestamp with the minimum distance(timewise).
        
        // TODO: use sec and nanosec for comparison
        while (abs((int)(lidar_timestamp - candidate.header.stamp.sec)) > abs((int)(lidar_timestamp - other_candidate.header.stamp.sec))) {
            // Slide forward candidate to the new oldest element
            candidate = other_candidate;

            // Pop the oldest element
            this->pop();

            // If the queue is empty, just return candidate
            if (queue_.size() == 0)
                return candidate;

            // Take the second oldest element
            other_candidate = queue_.front();
        }

        return candidate;
    }

private:
    std::queue<zed_msgs::msg::BoundingBoxList2d> queue_;
    std::size_t max_queue_size_;
};
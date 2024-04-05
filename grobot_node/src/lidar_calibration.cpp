#include "grobot_node/lidar_calibration.hpp"

//
//   created by: Masaaki Hijikata(hijimasa)
//   github.com/hijimasa
//

class LidarCalibrationNode : public rclcpp::Node
{
public:
  LidarCalibrationNode()
      : Node("lidar_calibration_node")
  {
    auto default_qos = rclcpp::SensorDataQoS(); 

    // Subscriber
    subscription_lidar_right =  this->create_subscription<sensor_msgs::msg::LaserScan>(
        "LIDAR1/scan", default_qos, std::bind(&LidarCalibrationNode::scan_callback_right, this, std::placeholders::_1));

    subscription_lidar_left =  this->create_subscription<sensor_msgs::msg::LaserScan>(
        "LIDAR2/scan", default_qos, std::bind(&LidarCalibrationNode::scan_callback_left, this, std::placeholders::_1));
    
    //Publisher
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SystemDefaultsQoS());
    // scan_pub1 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan1", 10);
    // scan_pub2 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 10);
    // scan_pub3 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan3", 10);
    // scan_pub4 = this->create_publisher<sensor_msgs::msg::LaserScan>("scan4", 10);


    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
  }

  void scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser1_ = msg;
    if(laser2_)
    { 
    //   RCLCPP_INFO(this->get_logger(), "22222222222");
      update_pointcloud();
    }
  }

  
  void scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    laser2_ = msg;

  }
  
  //라이다범위 나누는거 확인용 (사용x)
  void publish_filtered_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_min_deg, double angle_max_deg, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub) 
  {
    auto filtered_scan = *msg; 
    filtered_scan.ranges.clear();
    filtered_scan.ranges.resize(msg->ranges.size(), std::numeric_limits<float>::infinity());

    double angle_min = angle_min_deg * M_PI / 180.0;
    double angle_max = angle_max_deg * M_PI / 180.0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) 
    {
        double angle = msg->angle_min + i * msg->angle_increment;
        if (angle >= angle_min && angle <= angle_max) 
        {
            filtered_scan.ranges[i] = msg->ranges[i];
        }
    }

    pub->publish(filtered_scan);
  }

  void update_pointcloud() 
  {   
      trans1_ = tf2_->lookupTransform("merged_base_scan", laser1_->header.frame_id, rclcpp::Time(0));
      trans2_ = tf2_->lookupTransform("merged_base_scan", laser2_->header.frame_id, rclcpp::Time(0));

      double sensor1_roll, sensor1_pitch, sensor1_yaw, sensor2_roll, sensor2_pitch, sensor2_yaw;

      geometry_quat_to_rpy(&sensor1_roll, &sensor1_pitch, &sensor1_yaw, trans1_.transform.rotation);
      geometry_quat_to_rpy(&sensor2_roll, &sensor2_pitch, &sensor2_yaw, trans2_.transform.rotation);
      
      sensor1_yaw += laser1Yaw_;
      sensor2_yaw += laser2Yaw_;

      std::vector<std::array<float,2>> scan_data;
      int count = 0;
      float min_theta = 0;
      float max_theta = 0;
      
      //LIDAR1 (RIGHT)
      for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment)
      {
        std::array<float, 2> point;
            
        float laser_angle = i;

        //LIDAR1 & LIDAR2프레임 기준으로 pointcloud의 x,y값 구하기
        float temp_x = laser1_->ranges[count] * std::cos(laser_angle) ;
        float temp_y = laser1_->ranges[count] * std::sin(laser_angle) ;

        // Merged LIDAR 프레임 기준으로 pointcloud들을 x,y값으로 구하기 
        point[0] = temp_x * std::cos(sensor1_yaw) - temp_y * std::sin(sensor1_yaw); 
        point[0] += trans1_.transform.translation.x + laser1XOff_;
        point[1] = temp_x * std::sin(sensor1_yaw) + temp_y * std::cos(sensor1_yaw);
        point[1] += trans1_.transform.translation.y + laser1YOff_;
        count++;

        // std::cout << "Point " << count << ": (" << point[0] << ", " << point[1] << ")" << std::endl;
        
        // 이때 x,y값이 로봇의 내부라면 무시 
        if (point[0] < robotFrontEnd_ && point[0] > -robotRearEnd_ && point[1] < robotLeftEnd_ && point[1] > -robotRightEnd_)
        {    
            // cout << "ROBOT_AREA" << endl;
            continue;
        }
        
        float r_ = GET_R(point[0], point[1]);
        float theta_ = GET_THETA(point[0], point[1]);
        std::array<float,2> res_;
        res_[1] = r_;
        res_[0] = theta_;
        scan_data.push_back(res_);
        if(theta_ < min_theta)
        {
            min_theta = theta_;
        }
        if(theta_ > max_theta)
        {
            max_theta = theta_;
        }
      }
      
      // cout << "Max_theta : " <<  max_theta << "Min_theta" <<  min_theta <<  endl;

      //LIDAR2 (LEFT)
      count = 0;
      for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment)
      {
        std::array<float,2> point; 
        float laser_angle = i;

        float temp_x = laser2_->ranges[count] * std::cos(laser_angle) ;
        float temp_y = laser2_->ranges[count] * std::sin(laser_angle) ;
        point[0] = temp_x * std::cos(sensor2_yaw) - temp_y * std::sin(sensor2_yaw);
        point[0] += trans2_.transform.translation.x + laser2XOff_;
        point[1] = temp_x * std::sin(sensor2_yaw) + temp_y * std::cos(sensor2_yaw);
        point[1] += trans2_.transform.translation.y + laser2YOff_;
        count++;

        if (point[0] < robotFrontEnd_ && point[0] > -robotRearEnd_ && point[1] < robotLeftEnd_ && point[1] > -robotRightEnd_)
        {   
            // cout << "ROBOT_AREA" << endl;
            continue;
        }

        float r_ = GET_R(point[0], point[1]);
        float theta_ = GET_THETA(point[0], point[1]);
        std::array<float,2> res_;
        res_[1] = r_;
        res_[0] = theta_;
        scan_data.push_back(res_);
        if(theta_ < min_theta)
        {
            min_theta = theta_;
        }

        if(theta_ > max_theta)
        {
            max_theta = theta_;
        }
      }
    
      //배열에 들어가 있는 NAN 값 제거 
      scan_data.erase(
      std::remove_if(
          scan_data.begin(), 
          scan_data.end(),
          [](const std::array<float, 2>& val) { return std::isnan(val[0]) || std::isnan(val[1]); }
      ),
      scan_data.end()
      );   
    
      //scan_data 값 정렬
      std::sort(scan_data.begin(), scan_data.end(), [](std::array<float,2> a, std::array<float,2> b) {return a[0] < b[0];});	
    //   for (const auto& point : scan_data) {
    //       std::cout << "(" << point[0] << ", " << point[1] << ")\n";
    //   }
     
      //Merged LIDAR Scan
      auto merged_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>();
      merged_scan_->header.frame_id = "merged_base_scan";
      merged_scan_->header.stamp = laser1_->header.stamp;
      merged_scan_->angle_min = min_theta;
      merged_scan_->angle_max = max_theta;
      merged_scan_->angle_increment = laser1_->angle_increment;
      merged_scan_->time_increment = laser1_->time_increment;
      merged_scan_->scan_time = laser1_->scan_time;
      merged_scan_->range_min = laser1_->range_min;
      merged_scan_->range_max = laser1_->range_max;

      size_t i = 1;
      std::vector<float>temp_range;
      for (float angle = min_theta; angle < max_theta; angle += laser1_->angle_increment)
      {
          while (scan_data[i][0] < angle)
          {
              i++;
          }
        
          //interpolate 
          if (fabs(scan_data[i][1] - scan_data[i-1][1]) < 0.2f && (fabs(scan_data[i][0] - angle) < laser1_->angle_increment || fabs(scan_data[i-1][0] - angle) < laser1_->angle_increment))
          {
              float range = interpolate(scan_data[i-1][0], scan_data[i][0], scan_data[i-1][1], scan_data[i][1], angle);
              temp_range.push_back(range);
          }

          else
          {
              temp_range.push_back(std::numeric_limits<float>::infinity());
              // temp_range.push_back(0);
              
          }
      }

      merged_scan_->ranges = temp_range;
      scan_pub->publish(*merged_scan_);
  }
  
  float GET_R(float x, float y)
  {
      return sqrt(x*x + y*y);
  }

  float GET_THETA(float x, float y)
  {
      return atan2(y, x);
  }

  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
      
      return (magnitude_1 + (current_angle - angle_1) * ((magnitude_2 - magnitude_1)/(angle_2 - angle_1)));
  }

  void geometry_quat_to_rpy(double* roll, double* pitch, double* yaw, geometry_msgs::msg::Quaternion geometry_quat)
  {
      tf2::Quaternion quat;
      tf2::convert(geometry_quat, quat);
      tf2::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);  //rpy are Pass by Reference
  }

private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_right;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_left;

    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub1;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub2;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub3;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub4;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;

    geometry_msgs::msg::TransformStamped trans1_;
    geometry_msgs::msg::TransformStamped trans2_;
    
    //일단 0으로 사용 
    float laser1XOff_, laser1YOff_, laser1Yaw_;
    float laser2XOff_, laser2YOff_, laser2Yaw_;

    double angle_min_deg_right = 0.0;
    double angle_max_deg_right = 180.0;

    double angle_min_deg_left = -180.0;
    double angle_max_deg_left = 0.0;  

    float robotFrontEnd_ = 0.33;
    float robotRearEnd_ = 0.29;
    float robotRightEnd_ = 0.25;
    float robotLeftEnd_ = 0.25;

};

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   RCLCPP_INFO(rclcpp::get_logger("Node ON"), "@@@@@@@@@@@ LIDAR Calibration node START @@@@@@@@@@@");
//   auto node = std::make_shared<LidarCalibrationNode>();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(node);
//   executor.spin(); // 멀티스레드 실행기를 사용하여 spin

//   rclcpp::shutdown();
//   return 0;
// }


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}


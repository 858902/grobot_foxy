#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <map>
#include <algorithm>

using namespace std;

class WaypointDijkstraNode : public rclcpp::Node
{
public:
  WaypointDijkstraNode()
      : Node("waypoint_dijkstra_node")
  {
    
    // Subscriber
    subscription_waypoint_ = this->create_subscription<std_msgs::msg::String>(
        "waypoint_list_raw", 10,std::bind(&WaypointDijkstraNode::list_callback, this, std::placeholders::_1)); // 들려야하는 Waypoint list 

    load_distances_from_yaml();


  }
    
  //방문해야하는 waypoint list 받아오기
  void list_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream iss(msg->data);
    std::string s;
    waypoint_vec.clear(); 

    while (iss >> s)
    {
        waypoint_vec.push_back(s);
    }

    for (const auto& waypoint : waypoint_vec) {
        RCLCPP_INFO(this->get_logger(), "Waypoint: %s", waypoint.c_str());
    }
    initialize_distance_matrix();
  } 

private:

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint_;

    std::vector<std::string> waypoint_vec; // 경유지를 저장할 벡터
    std::map<std::string, double> distance_map; // 경유지 간 거리를 저장할 맵
    const int N = 11;
    const double inf = numeric_limits<double>::infinity(); //무한대 정의 
    vector<vector<double>> distanceMatrix;  
    // vector<vector<double>> dp;
    // double dp;

    std::string start;
    std::string end;

    // YAML파일 불러오기
    void load_distances_from_yaml() 
    {
        std::string package_name = "grobot_navigation2";

        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        std::string path = package_path + "/param/path_distance_warehouse.yaml";
        YAML::Node yaml_file = YAML::LoadFile(path);

        for(YAML::const_iterator it=yaml_file.begin(); it!=yaml_file.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            double value = it->second.as<double>();
            distance_map[key] = value;
            RCLCPP_INFO(this->get_logger(), "Loaded distance: %s -> %f meters", key.c_str(), value);
        }
    }

    //가중치 matrix 정의 -> distanceMatrix[출발지][도착지] = 가중치 값 
    void initialize_distance_matrix() 
    {
        distanceMatrix.assign(N, vector<double>(N, inf));
        // dp.assign(1 << N, vector<double>(N, inf));

        // 자기 자신으로 가는 경로의 가중치를 0으로 설정
        for (int i = 0; i < N; ++i) 
        {
            distanceMatrix[i][i] = 0;
        }

        for (auto& p : distance_map) 
        {
            // yaml파일에서 가져온 문자열에서 'distance_' 부분을 제거 (인덱싱 과정)
            std::string keyWithoutPrefix = p.first.substr(std::string("distance_").length());
            
            std::string start, end;

            // 첫 번째 숫자를 start로, 나머지 숫자를 end로 설정
            size_t i = 0;
            while(i < keyWithoutPrefix.size() && !isdigit(keyWithoutPrefix[i])) i++;
            size_t startIdx = i;
            while(i < keyWithoutPrefix.size() && isdigit(keyWithoutPrefix[i])) i++;
            std::string temp = keyWithoutPrefix.substr(startIdx, i - startIdx);

            if (!temp.empty()) 
            {
                start = temp[0]; 
                if (temp.size() > 1) 
                {
                    end = temp.substr(1);
                }
            }

            int start_idx = std::stoi(start);
            int end_idx = std::stoi(end);
            double distance = p.second;

            // 양방향 경로 설정
            distanceMatrix[start_idx][end_idx] = distance;
            distanceMatrix[end_idx][start_idx] = distance;
        }
        
        // 행렬 출력용
        for (int i = 0; i < distanceMatrix.size(); ++i) {
            for (int j = 0; j < distanceMatrix[i].size(); ++j) {
                if (distanceMatrix[i][j] == inf) {
                    cout << "inf ";
                } else {
                    cout << distanceMatrix[i][j] << " ";
                }
            }
            cout << "\n";
        }
    }



};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointDijkstraNode>());
  rclcpp::shutdown();
  return 0;
}
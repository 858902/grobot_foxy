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

// waypoint 경로 표기용 구조체
struct TSPResult
{
    double cost;
    vector<int> path;

    TSPResult(double cost, vector<int> path) : cost(cost), path(move(path)) {}
};

class WaypointDynamicProgrammingNode : public rclcpp::Node
{
public:
  WaypointDynamicProgrammingNode()
      : Node("waypoint_dynamic_programming_node")
  {
    
    // Subscriber
    subscription_waypoint_ = this->create_subscription<std_msgs::msg::String>(
        "waypoint_list_raw", 10,std::bind(&WaypointDynamicProgrammingNode::list_callback, this, std::placeholders::_1)); // 들려야하는 Waypoint list 

    // Publisher
    waypoint_list_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_list", 10);

    load_yaml();


  }
    
  //방문해야하는 waypoint list 받아오기
//   void list_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     std::istringstream iss(msg->data);
//     std::string s;
//     waypoint_vec.clear(); 
//     waypoint_indices.clear();

//     while (iss >> s)
//     {
//         waypoint_vec.push_back(s);
//         int waypoint_idx = std::stoi(s);
//         waypoint_indices.push_back(waypoint_idx);
//     }

//     for (const auto& waypoint : waypoint_vec) {
//         RCLCPP_INFO(this->get_logger(), "Waypoint: %s", waypoint.c_str());
//     }

//     initialize_distance_matrix();
//     std::cout << "최소 거리: " << calculate_optimal_path(distanceMatrix, waypoint_indices,11) << std::endl;
//   } 

    void list_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream iss(msg->data);
        std::string token;
        waypoint_vec.clear();
        waypoint_indices.clear();

        while (std::getline(iss, token, ',')) // 구분자 ','를 사용하여 문자열 분리
        {
            waypoint_vec.push_back(token);
            int waypoint_idx = token[0] - 'A' + 1; // 문자를 숫자로 변환 ('A' -> 1, 'B' -> 2, ...)
            waypoint_indices.push_back(waypoint_idx);
        }

        for (const auto& waypoint : waypoint_vec) {
            RCLCPP_INFO(this->get_logger(), "Waypoint: %s", waypoint.c_str());
        }

        initialize_distance_matrix();
        std::cout << "최소 거리: " << calculate_optimal_path(distanceMatrix, waypoint_indices, 11) << std::endl;
    }
private:

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint_;

    //Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_list_pub_;

    std::vector<std::string> waypoint_vec; // 경유지를 저장할 벡터
    std::vector<std::string> optimal_waypoint_vec; // 경유지를 저장할 벡터
    std::vector<int> waypoint_indices; //매핑용

    const int N = 11;
    const double inf = numeric_limits<double>::infinity(); //무한대 정의 
    std::map<std::string, double> distance_map; // 경유지 간 거리를 저장할 맵
    vector<vector<double>> distanceMatrix;  

    std::string start;
    std::string end;

    // YAML파일 불러오기
    void load_yaml() 
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
        
        // // 행렬 출력용
        // for (int i = 0; i < distanceMatrix.size(); ++i) {
        //     for (int j = 0; j < distanceMatrix[i].size(); ++j) 
        //     {
        //         if (distanceMatrix[i][j] == inf) 
        //         {
        //             cout << "inf ";
        //         } 
        //         else 
        //         {
        //             cout << distanceMatrix[i][j] << " ";
        //         }
        //     }
        //     cout << "\n";
        // }
    }

    // TSP 문제 해결
    //masked는 현재까지 방문한 지점, visited는 마지막으로 방문한 지점
    TSPResult tsp(int masked, int visited, vector<vector<TSPResult>>& dp, vector<vector<double>>& distanceMatrix, vector<int>& waypoint_indices, int N) 
    {
        // 모든 지점을 방문 ->  마지막 지점에서 출발지로 돌아가는 비용 반환
        if(masked == ((1 << waypoint_indices.size()) - 1)) 
        {   
            // 모든 지정된 경유지를 방문한 경우, 도착지점으로의 거리를 계산
            return TSPResult(distanceMatrix[visited][N-1], vector<int>{N-1});
        }

        // 이미 계산된 경우, 재계산하지 않음
        if(dp[masked][visited].cost != -1) 
        {
            return dp[masked][visited];
        }

        // 요거는 가능한 최소 비용을 찾기 위해 무한대로 초기화하는 용도 
        double ans = inf;
        vector<int> bestPath;

        // 모든 지점을 순회 -> 다음 방문 지점 결정
        for(int i = 0; i < waypoint_indices.size(); i++) 
        {   
            // 아직 방문 x 일경우 
            if((masked & (1 << i)) == 0) 
            {
                int nextCity = waypoint_indices[i];

                TSPResult nextResult = tsp(masked | (1 << i), nextCity, dp, distanceMatrix, waypoint_indices, N);
                // 현재 지점에서 다음 지점까지의 비용 + 다음 지점에서 시작하는 TSP의 결과를 계산
                double newAns = distanceMatrix[visited][nextCity] + nextResult.cost;

                // 최소 비용 업데이트
                if(newAns < ans) 
                {
                    ans = newAns;
                    bestPath = nextResult.path;
                    bestPath.insert(bestPath.begin(), nextCity);
                }
            }
        }

        // 계산된 최소 비용을 저장하고 반환
        dp[masked][visited] = TSPResult(ans, bestPath);
        return dp[masked][visited];
    }

    double calculate_optimal_path(vector<vector<double>>& distanceMatrix, vector<int>& waypoint_indices, int N) 
    {
        vector<vector<TSPResult>> dp(1 << waypoint_indices.size(), vector<TSPResult>(N, TSPResult(-1, {})));
        TSPResult result = tsp(0, 0, dp, distanceMatrix, waypoint_indices, N);

        // 경로 출력
        cout << "최소 비용으로 방문하는 경로: ";
        for(int i : result.path) 
        {
            cout << i << " ";
            optimal_waypoint_vec.push_back(std::to_string(i));
        }
        cout << endl;
        
        std::string waypoints_str;
        for(const auto& waypoint : optimal_waypoint_vec) 
        {
            waypoints_str += waypoint + " "; // 각 경유지 번호 뒤에 공백 추가
        }

        if (!waypoints_str.empty()) 
        {
            waypoints_str.pop_back();
        }

        std_msgs::msg::String msg;
        msg.data = waypoints_str;

        waypoint_list_pub_->publish(msg);

        return result.cost;
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointDynamicProgrammingNode>());
  rclcpp::shutdown();
  return 0;
}
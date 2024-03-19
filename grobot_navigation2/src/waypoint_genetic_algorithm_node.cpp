// #include "rclcpp/rclcpp.hpp"
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <yaml-cpp/yaml.h>
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_srvs/srv/set_bool.hpp"

// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// #include <string>
// #include <vector>
// #include <cmath>
// #include <limits>
// #include <map>
// #include <algorithm>
// #include <random>
// #include <iostream>

// using namespace std;

// // Waypoint 경로 표기용 구조체
// struct TSPResult
// {
//     double cost;
//     vector<int> path;

//     TSPResult(double cost, vector<int> path) : cost(cost), path(move(path)) {}
// };

// class WaypointGeneticAlgorithmNode : public rclcpp::Node
// {
// public:
//     WaypointGeneticAlgorithmNode()
//         : Node("waypoint_genetic_algorithm_node")
//     {
//         // Subscriber
//         subscription_waypoint_ = this->create_subscription<std_msgs::msg::String>(
//             "waypoint_list_raw", 10, std::bind(&WaypointGeneticAlgorithmNode::list_callback, this, std::placeholders::_1)); // 들려야하는 Waypoint list

//         // Publisher
//         waypoint_list_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_list", 10);

//         load_yaml();
//         gen = mt19937(rd());
//     }

//     // 방문해야하는 waypoint list 받아오기
//     void list_callback(const std_msgs::msg::String::SharedPtr message)
//     {
//         std::istringstream iss(message->data);
//         std::string s;
//         waypoint_vec.clear();
//         waypoint_indices.clear();

//         while (iss >> s)
//         {
//             waypoint_vec.push_back(s);
//             int waypoint_idx = std::stoi(s);
//             waypoint_indices.push_back(waypoint_idx);
//         }

//         for (const auto& waypoint : waypoint_vec) {
//             RCLCPP_INFO(this->get_logger(), "Waypoint: " + waypoint);
//         }

//         initialize_distance_matrix();
        
//         TSPResult result = solve_genetic_algorithm(distanceMatrix, waypoint_indices, N);
//         RCLCPP_INFO(get_logger(), "최소 거리: %f", result.cost);

//         std::string waypoints_str;
//         for (int i : result.path)
//         {
//             waypoints_str += std::to_string(i) + " ";
//         }

//         std_msgs::msg::String result_msg;
//         result_msg.data = waypoints_str;

//         waypoint_list_pub_->publish(result_msg);
//     }
// private:
//     // Subscriber
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint_;

//     // Publisher
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_list_pub_;

//     std::vector<std::string> waypoint_vec; // 경유지를 저장할 벡터
//     std::vector<int> waypoint_indices; // 매핑용

//     random_device rd;
//     mt19937 gen;

//     const int N = 11;
//     const double inf = numeric_limits<double>::infinity(); // 무한대 정의
//     std::map<std::string, double> distance_map;           // 경유지 간 거리를 저장할 맵
//     vector<vector<double>> distanceMatrix;

//     // YAML 파일 불러오기
//     void load_yaml()
//     {
//         std::string package_name = "grobot_navigation2";

//         std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
//         std::string path = package_path + "/param/path_distance_warehouse.yaml";
//         YAML::Node yaml_file = YAML::LoadFile(path);

//         for (YAML::const_iterator it = yaml_file.begin(); it != yaml_file.end(); ++it)
//         {
//             std::string key = it->first.as<std::string>();
//             double value = it->second.as<double>();
//             distance_map[key] = value;
//             RCLCPP_INFO_STREAM(get_logger(), "Loaded distance: " << key << " -> " << value << " meters");
//         }
//     }

//     // 가중치 matrix 정의 -> distanceMatrix[출발지][도착지] = 가중치 값
//     void initialize_distance_matrix()
//     {
//         distanceMatrix.assign(N, vector<double>(N, inf));

//         // 자기 자신으로 가는 경로의 가중치를 0으로 설정
//         for (int i = 0; i < N; ++i)
//         {
//             distanceMatrix[i][i] = 0;
//         }

//         for (auto &p : distance_map)
//         {
//             // yaml파일에서 가져온 문자열에서 'distance_' 부분을 제거 (인덱싱 과정)
//             std::string keyWithoutPrefix = p.first.substr(std::string("distance_").length());

//             std::string start, end;

//             // 첫 번째 숫자를 start로, 나머지 숫자를 end로 설정
//             size_t i = 0;
//             while (i < keyWithoutPrefix.size() && !isdigit(keyWithoutPrefix[i]))
//                 i++;
//             size_t startIdx = i;
//             while (i < keyWithoutPrefix.size() && isdigit(keyWithoutPrefix[i]))
//                 i++;
//             std::string temp = keyWithoutPrefix.substr(startIdx, i - startIdx);

//             if (!temp.empty())
//             {
//                 start = temp[0];
//                 if (temp.size() > 1)
//                 {
//                     end = temp.substr(1);
//                 }
//             }

//             int start_idx = std::stoi(start);
//             int end_idx = std::stoi(end);
//             double distance = p.second;

//             // 양방향 경로 설정
//             distanceMatrix[start_idx][end_idx] = distance;
//             distanceMatrix[end_idx][start_idx] = distance;
//         }
//     }

//     // 유전 알고리즘을 사용하여 TSP 문제 해결
//     TSPResult solve_genetic_algorithm(vector<vector<double>> &distanceMatrix, vector<int> &waypoint_indices, int N)
//     {
//         const int population_size = 100;
//         const int generations = 1000;
//         const double mutation_rate = 0.01;

//         vector<vector<int>> population(population_size, waypoint_indices);
        
//         std::random_device rd;
//         std::mt19937 rng(rd());
//         std::uniform_int_distribution<size_t> int_dis(0, population_size - 1);
//         std::uniform_real_distribution<double> real_dis(0.0, 1.0);

//         for (int gen = 0; gen < generations; ++gen)
//         {
//             // 적합도 평가
//             vector<pair<double, vector<int>>> fitness_values;
//             for (auto &individual : population)
//             {
//                 double fitness = calculate_path_cost(distanceMatrix, individual);
//                 fitness_values.push_back({fitness, individual});
//             }

//             // 적합도 기준으로 정렬
//             sort(fitness_values.begin(), fitness_values.end());

//             // 다음 세대 생성
//             vector<vector<int>> next_generation(population_size);
//             for (int i = 0; i < population_size; ++i)
//             {
//                 // 선택: 토너먼트 선택(Tournament selection)
//                 size_t idx1 = int_dis(rng);
//                 size_t idx2 = int_dis(rng);
//                 // size_t idx1 = int_dis(gen);
//                 // size_t idx2 = int_dis(gen);
//                 vector<int> parent1 = fitness_values[idx1].second;
//                 vector<int> parent2 = fitness_values[idx2].second;

//                 // 교차: 순서 교차(Ordinal crossover)
//                 vector<int> child = order_crossover(parent1, parent2);

//                 // 돌연변이: 일정 확률로 돌연변이 발생
//                 // if (real_dis(gen) < mutation_rate)
//                 // {
//                 //     mutate(child);
//                 // }
//                 if (real_dis(rng) < mutation_rate)
//                 {
//                     mutate(child);
//                 }

//                 next_generation[i] = child;
//             }

//             population = next_generation;
//         }

//         // 최적 해 찾기
//         double min_cost = numeric_limits<double>::infinity();
//         vector<int> optimal_path;
//         for (auto &individual : population)
//         {
//             double cost = calculate_path_cost(distanceMatrix, individual);
//             if (cost < min_cost)
//             {
//                 min_cost = cost;
//                 optimal_path = individual;
//             }
//         }

//         return {min_cost, optimal_path};
//     }


//     // 경로 비용 계산
//     double calculate_path_cost(vector<vector<double>> &distanceMatrix, vector<int> &path)
//     {
//         double cost = 0;
//         for (int i = 0; i < path.size() - 1; ++i)
//         {
//             cost += distanceMatrix[path[i]][path[i + 1]];
//         }
//         cost += distanceMatrix[path.back()][N - 1]; // 도착지까지의 거리 추가
//         return cost;
//     }

//     // 순서 교차
//     vector<int> order_crossover(vector<int> &parent1, vector<int> &parent2)
//     {
//         int n = parent1.size();
//         std::uniform_int_distribution<int> dis(0, n - 1);

//         int start = dis(gen);
//         int end = dis(gen);
//         if (start > end)
//         {
//             swap(start, end);
//         }

//         vector<int> child(n, -1);
//         for (int i = start; i <= end; ++i)
//         {
//             child[i] = parent1[i];
//         }

//         int idx = 0;
//         for (int i = 0; i < n; ++i)
//         {
//             if (child[i] == -1)
//             {
//                 while (find(child.begin(), child.end(), parent2[idx]) != child.end())
//                 {
//                     ++idx;
//                 }
//                 child[i] = parent2[idx++];
//             }
//         }

//         return child;
//     }

//     // 돌연변이
//     void mutate(vector<int> &individual)
//     {
//         std::uniform_int_distribution<int> dis(0, individual.size() - 1);
//         int idx1 = dis(gen);
//         int idx2 = dis(gen);
//         swap(individual[idx1], individual[idx2]);
//     }

// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     rclcpp::spin(std::make_shared<WaypointGeneticAlgorithmNode>());
//     rclcpp::shutdown();
//     return 0;
// }


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
#include <random>

using namespace std;

// Waypoint 경로 표기용 구조체
struct TSPResult
{
    double cost;
    vector<int> path;

    TSPResult(double cost, vector<int> path) : cost(cost), path(move(path)) {}
};

class WaypointGeneticAlgorithmNode : public rclcpp::Node
{
public:
    WaypointGeneticAlgorithmNode()
        : Node("waypoint_genetic_algorithm_node")
    {
        // Subscriber
        subscription_waypoint_ = this->create_subscription<std_msgs::msg::String>(
            "waypoint_list_raw", 10, std::bind(&WaypointGeneticAlgorithmNode::list_callback, this, std::placeholders::_1)); // 들려야하는 Waypoint list

        // Publisher
        waypoint_list_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_list", 10);

        load_yaml();
    }

    //lsit callback temp1
    void list_callback(const std_msgs::msg::String::SharedPtr message)
    {
        std::istringstream iss(message->data);
        std::string s;
        waypoint_vec.clear();
        waypoint_indices.clear();

        std::set<int> unique_waypoints; // 중복ㅈ거
        waypoint_vec.push_back("0");
        waypoint_indices.push_back(0);

        while (iss >> s)
        {
            int waypoint_idx = std::stoi(s);
            if (waypoint_idx != 0 && waypoint_idx != 10 && unique_waypoints.find(waypoint_idx) == unique_waypoints.end()) {
                waypoint_vec.push_back(s);
                waypoint_indices.push_back(waypoint_idx);
                unique_waypoints.insert(waypoint_idx);
            }
        }

        waypoint_vec.push_back("10");
        waypoint_indices.push_back(10);

        RCLCPP_INFO(this->get_logger(), "Raw list: %s", message->data.c_str());


        // 유전 알고리즘을 사용하여 최적 경로 계산
        initialize_distance_matrix();
        TSPResult result = solve_genetic_algorithm(distanceMatrix, waypoint_indices, 11);
        RCLCPP_INFO(get_logger(), "최소 거리: %f", result.cost);

        // 최적 경로 출력
        // std::string optimal_path_str;
        std::string optimal_path_str = "0 ";
        for (int i : result.path)
        {
            if(i != 0 && i != 10){
                optimal_path_str += std::to_string(i) + " ";
            }
        }
        optimal_path_str += "10"; // 목적지 추가
        RCLCPP_INFO(get_logger(), "Optimal path: %s", optimal_path_str.c_str());

        std_msgs::msg::String result_msg;
        result_msg.data = optimal_path_str;

        waypoint_list_pub_->publish(result_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing message: %s", result_msg.data.c_str());
    }

    // // 방문해야하는 waypoint list 받아오기
    // void list_callback(const std_msgs::msg::String::SharedPtr message)
    // {
    //     std::istringstream iss(message->data);
    //     std::string s;
    //     waypoint_vec.clear();
    //     waypoint_indices.clear();

    //     // 웨이포인트 중복 제거를 위해 set을 사용합니다.
    //     std::set<int> unique_waypoints;

    //     while (iss >> s)
    //     {
    //         int waypoint_idx = std::stoi(s);
    //         // 중복을 방지하기 위해 set에 추가합니다.
    //         if (waypoint_idx != 0 && waypoint_idx != 10 && unique_waypoints.find(waypoint_idx) == unique_waypoints.end()) {
    //             waypoint_vec.push_back(s);
    //             waypoint_indices.push_back(waypoint_idx);
    //             unique_waypoints.insert(waypoint_idx);
    //         }
    //     }

    //     // 시작점과 끝점을 확실히 추가합니다.
    //     waypoint_vec.insert(waypoint_vec.begin(), "0");
    //     waypoint_indices.insert(waypoint_indices.begin(), 0);
    //     waypoint_vec.push_back("10");
    //     waypoint_indices.push_back(10);

    //     RCLCPP_INFO(this->get_logger(), "raw Waypoint 리스트:");
    //     for (const auto& waypoint : waypoint_vec) {
    //         RCLCPP_INFO(this->get_logger(), "Waypoint: " + waypoint);
    //     }

    //     initialize_distance_matrix();
        
    //     TSPResult result = solve_genetic_algorithm(distanceMatrix, waypoint_indices, N);
    //     RCLCPP_INFO(get_logger(), "최소 거리: %f", result.cost);

    //     std::string waypoints_str;
    //     for (int i : result.path)
    //     {
    //         waypoints_str += std::to_string(i) + " ";
    //     }

    //     std_msgs::msg::String result_msg;
    //     result_msg.data = waypoints_str;

    //     waypoint_list_pub_->publish(result_msg);
    // }
private:
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_list_pub_;

    std::vector<std::string> waypoint_vec; // 경유지를 저장할 벡터
    std::vector<int> waypoint_indices; // 매핑용

    random_device rd;
    mt19937 gen;

    const int N = 11;
    const double inf = numeric_limits<double>::infinity(); // 무한대 정의
    std::map<std::string, double> distance_map;           // 경유지 간 거리를 저장할 맵
    vector<vector<double>> distanceMatrix;

    // YAML 파일 불러오기
    void load_yaml()
    {
        std::string package_name = "grobot_navigation2";

        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        std::string path = package_path + "/param/path_distance_warehouse.yaml";
        YAML::Node yaml_file = YAML::LoadFile(path);

        for (YAML::const_iterator it = yaml_file.begin(); it != yaml_file.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            double value = it->second.as<double>();
            distance_map[key] = value;
            RCLCPP_INFO(get_logger(), "Loaded distance: %s -> %f meters", key.c_str(), value);
        }
    }

    // 가중치 matrix 정의 -> distanceMatrix[출발지][도착지] = 가중치 값
    void initialize_distance_matrix()
    {
        distanceMatrix.assign(N, vector<double>(N, inf));

        // 자기 자신으로 가는 경로의 가중치를 0으로 설정
        for (int i = 0; i < N; ++i)
        {
            distanceMatrix[i][i] = 0;
        }

        for (auto &p : distance_map)
        {
            // yaml파일에서 가져온 문자열에서 'distance_' 부분을 제거 (인덱싱 과정)
            std::string keyWithoutPrefix = p.first.substr(std::string("distance_").length());

            std::string start, end;

            // 첫 번째 숫자를 start로, 나머지 숫자를 end로 설정
            size_t i = 0;
            while (i < keyWithoutPrefix.size() && !isdigit(keyWithoutPrefix[i]))
                i++;
            size_t startIdx = i;
            while (i < keyWithoutPrefix.size() && isdigit(keyWithoutPrefix[i]))
                i++;
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
    }

    // 유전 알고리즘
    TSPResult solve_genetic_algorithm(vector<vector<double>> &distanceMatrix, vector<int> &waypoint_indices, int N)
    {
        const int population_size = 100;
        const int generations = 100;
        const double mutation_rate = 0.01;

        // 출발지와 목적지를 고정
        vector<int> fixed_waypoints = {0, N - 1}; // 출발지와 목적지
        vector<int> fixed_indices = {0, N - 1};

        // 경유지를 제외한 부분을 업데이트
        for (int i : waypoint_indices) {
            if (i != 0 && i != N - 1) {
                fixed_waypoints.push_back(i);
                fixed_indices.push_back(i);
            }
        }

        // vector<vector<int>> population(population_size, waypoint_indices);
        vector<vector<int>> population(population_size, fixed_indices);
        
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<size_t> int_dis(0, population_size - 1);
        std::uniform_real_distribution<double> real_dis(0.0, 1.0);

        for (int gen = 0; gen < generations; ++gen)
        {
            // fitness check
            vector<pair<double, vector<int>>> fitness_values;
            for (auto &individual : population)
            {
                double fitness = calculate_path_cost(distanceMatrix, individual);
                fitness_values.push_back({fitness, individual});
            }

            // sort ㄱㄱ
            sort(fitness_values.begin(), fitness_values.end());

            // next gen
            vector<vector<int>> next_generation(population_size);
            for (int i = 0; i < population_size; ++i)
            {
                // 토너먼트 실행
                size_t idx1 = int_dis(rng);
                size_t idx2 = int_dis(rng);
                vector<int> parent1 = fitness_values[idx1].second;
                vector<int> parent2 = fitness_values[idx2].second;

                // 교차
                vector<int> child = order_crossover(parent1, parent2);

                // 돌연변이
                if (real_dis(rng) < mutation_rate)
                {
                    mutate(child);
                }

                next_generation[i] = child;
            }

            population = next_generation;
        }

        // optimal cost 
        double min_cost = numeric_limits<double>::infinity();
        vector<int> optimal_path;
        for (auto &individual : population)
        {
            double cost = calculate_path_cost(distanceMatrix, individual);
            if (cost < min_cost)
            {
                min_cost = cost;
                optimal_path = individual;
            }
        }

        return {min_cost, optimal_path};
    }


    // 경로 설ㅈ정
    double calculate_path_cost(vector<vector<double>> &distanceMatrix, vector<int> &path)
    {
        double cost = 0;
        for (int i = 0; i < path.size() - 1; ++i)
        {
            cost += distanceMatrix[path[i]][path[i + 1]];
        }
        cost += distanceMatrix[path.back()][N - 1]; // 도착지까지의 거리 추가
        return cost;
    }

    // 순서 교차
    vector<int> order_crossover(vector<int> &parent1, vector<int> &parent2)
    {
        int n = parent1.size();
        std::uniform_int_distribution<int> dis(0, n - 1);

        int start = dis(gen);
        int end = dis(gen);
        if (start > end)
        {
            swap(start, end);
        }

        vector<int> child(n, -1);
        for (int i = start; i <= end; ++i)
        {
            child[i] = parent1[i];
        }

        int idx = 0;
        for (int i = 0; i < n; ++i)
        {
            if (child[i] == -1)
            {
                while (find(child.begin(), child.end(), parent2[idx]) != child.end())
                {
                    ++idx;
                }
                child[i] = parent2[idx++];
            }
        }

        return child;
    }

    // 돌연변이
    void mutate(vector<int> &individual)
    {
        std::uniform_int_distribution<int> dis(0, individual.size() - 1);
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        swap(individual[idx1], individual[idx2]);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<WaypointGeneticAlgorithmNode>());
    rclcpp::shutdown();
    return 0;
}

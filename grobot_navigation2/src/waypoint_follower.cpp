#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <vector>
#include <unordered_map>

#include <mutex>
#include <chrono>
#include <condition_variable>
#include <atomic>

std::mutex mutex_;
using namespace std;

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
      : Node("waypoint_follower_node")
  {
    
    // Subscriber
    subscription_waypoint = this->create_subscription<std_msgs::msg::String>(
        "waypoint_list", 10,std::bind(&WaypointFollowerNode::waypoint_list_callback, this, std::placeholders::_1)); // 들려야하는 Waypoint list 

    subscription_signal_ = this->create_subscription<std_msgs::msg::String>(
        "start_signal", 10, std::bind(&WaypointFollowerNode::signal_callback, this, std::placeholders::_1));  //다음 Waypoint로 이동할지 여부 판단 (의사판단 노드로 부터 String 형태로 sub) 

    // Publisher
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10); //시작지점 

    // Action_Client_
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this, "/FollowWaypoints"); //waypoint folllower action 

    
    // Set Initial Position 
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();
    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;

    initial_pose_pub_->publish(initial_pose);

    // Define waypoints
    waypoints_["A"].header.frame_id = "map";
    waypoints_["A"].pose.position.x = 8.0;
    waypoints_["A"].pose.position.y = -1.0;
    waypoints_["A"].pose.orientation.w = 1.0;

    waypoints_["B"].header.frame_id = "map";
    waypoints_["B"].pose.position.x = 3.0;
    waypoints_["B"].pose.position.y = 3.0;
    waypoints_["B"].pose.orientation.w = 1.0;

    waypoints_["C"].header.frame_id = "map";
    waypoints_["C"].pose.position.x = 0.0;
    waypoints_["C"].pose.position.y = -5.0;
    waypoints_["C"].pose.orientation.w = 1.0;

    load_yaml_();

  }
    void load_yaml_() 
    {
        std::string package_name = "grobot_navigation2";

        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        std::string path = package_path + "/param/waypoint_warehouse.yaml";
        YAML::Node yaml_file = YAML::LoadFile(path);

        for(YAML::const_iterator it=yaml_file.begin(); it!=yaml_file.end(); ++it)
        {
            std::string waypoint_name = it->first.as<std::string>();
            double x = it->second[0].as<double>();
            double y = it->second[1].as<double>();
            double yaw = it->second[2].as<double>();

            std::string id = waypoint_name.substr(waypoint_name.find("_") + 1);

            waypoints_[id].header.frame_id = "map";
            waypoints_[id].pose.position.x = x;
            waypoints_[id].pose.position.y = y;
            waypoints_[id].pose.orientation.w = 1.0;

            RCLCPP_INFO(this->get_logger(), "Waypoint loaded: %s, Position: (%f, %f), Orientation: %f", id.c_str(), x, y, yaw);

        }
    }

    void waypoint_list_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream iss(msg->data);
        std::string s;
        while (iss >> s)
        {
            waypoint_keys_.push_back(s);
        }
    }

    void send_goal_to_current_waypoint()
    {
        if (current_waypoint_index_ >= waypoint_keys_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been processed");
            return;
        }

        if (!signal_received_)
        {
            return; 
        }

        std::string key = waypoint_keys_[current_waypoint_index_];
        auto it = waypoints_.find(key);
        if (it == waypoints_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Waypoint not found: %s", key.c_str());
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> waypoints_to_send = {it->second};
        send_goal(waypoints_to_send);

        signal_received_ = false; // 변수 초기화 
        current_waypoint_index_++; // 다음 waypoint로 업데이트 
    }

    void send_goal(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) 
    {
      // Navigation goal 정의 
      auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
      goal_msg.poses = waypoints;

      if (waypoints.empty())
      {
          RCLCPP_ERROR(this->get_logger(), "Waypoints array is empty");
          return;
      }

      if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) 
      {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
      }

      // Send the goal
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
      // rclcpp::Rate r(50);
      
      for (const auto& waypoint : waypoints)
      {   

          // 각 waypoint에 대하여 목표를 설정하고 전송
          goal_msg.poses.clear();
          goal_msg.poses.push_back(waypoint);
          auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

          RCLCPP_INFO(this->get_logger(), "Waypoint: %f, %f, %f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);

          RCLCPP_INFO(this->get_logger(), "signal_received: %s", signal_received_ ? "true" : "false");

          //Debug용 로그
          if (future_goal_handle.wait_for(std::chrono::seconds(10)) == std::future_status::ready) 
          {
            RCLCPP_INFO(this->get_logger(), "Waypoint Goal 전송완료 ");
          } 
          
          else if (future_goal_handle.wait_for(std::chrono::seconds(0)) == std::future_status::timeout) 
          {
            RCLCPP_INFO(this->get_logger(), "Waypoint Goal 대기중 ");
          } 
          
          else 
          {
            RCLCPP_INFO(this->get_logger(), "Waypoint 응답없음");
          }

      }

    }

  void signal_callback(const std_msgs::msg::String::SharedPtr msg)
  {
      if(msg->data == "ok") 
      { 
          signal_received_ = true;
          send_goal_to_current_waypoint();
      }
  }

  
private:
  
  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_signal_;

  //Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;


  // Action Client
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr action_client_;


  // Waypoints
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  bool signal_received_ = false;
  size_t current_waypoint_index_ = 0; // 현재 웨이포인트 인덱스
  std::vector<std::string> waypoint_keys_;


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
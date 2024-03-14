#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <string>
#include <vector>
#include <unordered_map>

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode()
      : Node("waypoint_follower_node")
  {

    // Subscriber
    // subscription_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //   "goal", 10, std::bind(&WaypointFollowerNode::goal_callback, this, std::placeholders::_1));
    subscription_waypoint = this->create_subscription<std_msgs::msg::String>(
        "waypoint_list", 10,std::bind(&WaypointFollowerNode::waypoint_list_callback, this, std::placeholders::_1));

    // Publisher_
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    // Action_Client_
    // action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    //     this, "/navigate_to_pose");
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this, "/FollowWaypoints");

    
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


    waypoints_["O"].header.frame_id = "map";
    waypoints_["O"].pose.position.x = 0.0;
    waypoints_["O"].pose.position.y = 0.0;
    waypoints_["O"].pose.orientation.w = 1.0;


  }
  
  // 단일 Goal 보내는 용도 
  // void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  // {
  //   // Define the goal
  //   auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  //   goal_msg.pose = *msg;

  //   // Send the goal
  //   auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  //   auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

  //   // Wait for the result
  //   if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) 
  //       != rclcpp::FutureReturnCode::SUCCESS)
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Send goal call failed :(");
  //     return;
  //   }

  //   rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = future_goal_handle.get();
  //   if (!goal_handle) {
  //     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  //     return;
  //   }
  // } 

  void waypoint_list_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Waypoint list 분류 (나중에 바꿀 수도 있음)
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::istringstream iss(msg->data);
    for (std::string s; iss >> s;)
    {
      auto it = waypoints_.find(s);
      if (it != waypoints_.end())
      {
        waypoints.push_back(it->second);
      }

      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown waypoint: %s", s.c_str());
        return;
      }
    }
    
  
    // Navigation goal 정의 
    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses = waypoints;

    if (waypoints.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Waypoints array is empty");
        return;
    }

    for (const auto& waypoint : waypoints)
    {
        RCLCPP_INFO(this->get_logger(), "Waypoint: %f, %f, %f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }
    // Send the goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (future_goal_handle.wait_for(std::chrono::seconds(10)) == std::future_status::ready) 
    {
      RCLCPP_INFO(this->get_logger(), "Goal 전송완료 ");
    } 
    
    else if (future_goal_handle.wait_for(std::chrono::seconds(0)) == std::future_status::timeout) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal 대기중 ");
    } 
    
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "대기중");
    }
  }

private:
  
  // Subscriber
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_goal;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_waypoint;


  //Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  // Action Client
  // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr action_client_;


   // Waypoints
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
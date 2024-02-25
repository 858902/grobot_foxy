#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class NavigateControllerNode : public rclcpp::Node
{
public:
  NavigateControllerNode()
      : Node("navigate_controller_node")
  {

    //Subscription_
    goal_keyword_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "goal_keyword", 10, std::bind(&NavigateControllerNode::cbGoalSelect, this, std::placeholders::_1));

    human_detect_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "human_detect", 10, std::bind(&NavigateControllerNode::cbHumanDetect, this, std::placeholders::_1));

    // Publisher_
    initial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    goal_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("goal_status", 10);

    // Action_Client_
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "/navigate_to_pose");

    // Service Client initialization
    gesture_client_ = this->create_client<std_srvs::srv::SetBool>("gesture_service");

    // initial_pose 설정
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

    initial_pose_pub->publish(initial_pose);
  }

private:

  void cbHumanDetect(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "human_following")
    {
      // 여기에 사람 따라가기 node를 실행하는 코드를 추가
      // 실행 후 대기하고 키워드를 받을 준비
    }
  }

  void cbGoalSelect(const std_msgs::msg::String::SharedPtr msg)
  {
    // 사람 따라가기 node를 종료
    // 목적지를 설정하고 이동
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    if (msg->data == "product1")
    {
      goal_msg.pose.pose.position.x = 0.1;
      goal_msg.pose.pose.position.y = 1.0;
    //   goal_msg.pose.pose.orientation.w = 0.0;
    }
    else if (msg->data == "product2")
    { 
      goal_msg.pose.pose.position.x = 2.0;
      goal_msg.pose.pose.position.y = 2.5;
    }
    else if (msg->data == "product3")
    {
      goal_msg.pose.pose.position.x = 2.0;
      goal_msg.pose.pose.position.y = -1.5;
    }
    else if (msg->data == "product4")
    {
      goal_msg.pose.pose.position.x = 4.0;
      goal_msg.pose.pose.position.y = 0.1;
    }

    else if (msg->data == "origin")
    {
      goal_msg.pose.pose.position.x = 0.0;
      goal_msg.pose.pose.position.y = 0.0;
    }

    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid location");
      return;
    }

    // Add quaternion
    double theta = 0.0; // 원하는 theta값 설정
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation = tf2::toMsg(quat);

    // Wait for server
    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to become available...");
    }

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigateControllerNode::result_callback, this, std::placeholders::_1);
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  }

void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    std_msgs::msg::Bool goal_status_msg;

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            goal_status_msg.data = true;
            RCLCPP_INFO(this->get_logger(), "Arrived at the goal");

            // 여기에 안내 시작 node를 실행하는 코드를 추가

            // 안내가 끝난 후 인사 node를 실행
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;  // 인사 node를 실행
            auto result_future = gesture_client_->async_send_request(request, 
                [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Gesture node started");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", response->message.c_str());
                    }
                });
        }
        break;

        default:
        {
            goal_status_msg.data = false;
            RCLCPP_INFO(this->get_logger(), "Failed to arrive at the goal");
        }
        break;
    }
    goal_status_publisher_->publish(goal_status_msg);
}

  //Subscription
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_keyword_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr human_detect_subscription_;

  //Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_status_publisher_;

  // Service Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gesture_client_;

  // Action Client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateControllerNode>());
  rclcpp::shutdown();
  return 0;
}

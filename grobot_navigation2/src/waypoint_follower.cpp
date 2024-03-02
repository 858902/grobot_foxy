#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class WaypointFollowerClient
{
public:
  WaypointFollowerClient()
  : node_(rclcpp::Node::make_shared("waypoint_follower_client"))
  {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "NavigateToPose");
  }

  void send_goal(const geometry_msgs::msg::PoseStamped& waypoint)
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = waypoint;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    // Here you can set callbacks for the goal response and feedback, if needed.

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto waypoint_follower_client = std::make_shared<WaypointFollowerClient>();

  // Here you should create your waypoints and send them to the waypoint follower.
  // Example:
  auto waypoint = geometry_msgs::msg::PoseStamped();
  waypoint.header.frame_id = "map";
  waypoint.pose.position.x = 1.0;
  waypoint.pose.position.y = 1.0;
  waypoint.pose.orientation.w = 1.0;

  waypoint_follower_client->send_goal(waypoint);

  rclcpp::spin(waypoint_follower_client->get_node());

  rclcpp::shutdown();
  return 0;
}
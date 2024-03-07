#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "std_msgs/msg/bool.hpp"

class PathLengthCalculator : public rclcpp::Node
{
public:
    PathLengthCalculator() : Node("path_length_calculator"), path_calculated_(false)
    {
        // Subscriber
        subscription_path = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathLengthCalculator::pathCallback, this, std::placeholders::_1));
        
        // Publisher_
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Action_Client_
        action_client_path = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this,"compute_path_to_pose");
    }

    void setInitialPose()
    {
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
    }

    void addGoal(double x, double y, double z, const std::string & goal_name)
    {
        path_calculated_ = false;

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = goal_name;
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.position.z = z;

        // 이전 웨이포인트와의 거리 계산
        if (!waypoints_.empty())
        {
            const auto& last_pose = waypoints_.back().pose;
            double dx = last_pose.position.x - x;
            double dy = last_pose.position.y - y;
            double dz = last_pose.position.z - z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            RCLCPP_INFO(this->get_logger(), "Distance from %s to %s: %.2f meters",
                        waypoints_.back().header.frame_id.c_str(), goal_name.c_str(), distance);
        }

        waypoints_.push_back(goal_pose); // 웨이포인트 목록에 추가

        computePath(goal_pose, goal_name);

        // 경로 계산이 완료될 때까지 기다림
        while (!path_calculated_) {
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    void calculateFinalDistance()
    {
        if (waypoints_.size() > 1) {
            const auto& first_pose = waypoints_.front().pose;
            const auto& last_pose = waypoints_.back().pose;

            double dx = last_pose.position.x - first_pose.position.x;
            double dy = last_pose.position.y - first_pose.position.y;
            double dz = last_pose.position.z - first_pose.position.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            RCLCPP_INFO(this->get_logger(), "Distance from %s to %s: %.2f meters",
                        waypoints_.back().header.frame_id.c_str(), waypoints_.front().header.frame_id.c_str(), distance);
        }
    }

private:
    void computePath(const geometry_msgs::msg::PoseStamped & goal_pose, const std::string & goal_name)
    {
        goal_name_ = goal_name;
        auto goal = nav2_msgs::action::ComputePathToPose::Goal();
        goal.pose = goal_pose;

        auto future = action_client_path->async_send_goal(goal);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        double total_length = 0.0;

        for (size_t i = 1; i < msg->poses.size(); ++i)
        {
            double dx = msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x;
            double dy = msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y;
            double dz = msg->poses[i].pose.position.z - msg->poses[i-1].pose.position.z;

            total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        RCLCPP_INFO(this->get_logger(), "Total path length to %s: %.2f meters", goal_name_.c_str(), total_length);
        path_calculated_ = true; // 경로 계산 완료
    }

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path;

    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    // Action_Client_
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_path;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    std::string goal_name_;
    bool path_calculated_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathLengthCalculator>();

    // node->setInitialPose(); // 초기 위치 설정
    
    node->addGoal(8.0, -1.0, 0.0, "A"); 
    node->addGoal(3.0, 3.0, 0.0, "B"); 
    node->addGoal(0.0, -5.0, 0.0, "C"); 

    node->calculateFinalDistance(); // 마지막 웨이포인트에서 첫 번째 웨이포인트로의 거리 계산

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
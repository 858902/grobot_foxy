#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp" 
#include <functional> 

using namespace std;

class VelocityManagerNode : public rclcpp::Node
{
    
public:
    VelocityManagerNode(): Node("velocity_manager")
    {   
        //Subscribers
        subscription_navigation = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_navigation", rclcpp::SystemDefaultsQoS(), std::bind(&VelocityManagerNode::navigation_velocity_callback, this, std::placeholders::_1));

        subscription_admittance = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_admittance", rclcpp::SystemDefaultsQoS(), std::bind(&VelocityManagerNode::admittance_velocity_callback, this, std::placeholders::_1));


        // Publisher 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
        
    }

    void navigation_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), "navigation_velocity_callback called");
        navigation_velocity_ = *msg;


    }

    void admittance_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), "admittance_velocity_callback called");
        admittance_velocity_ = *msg;


        if (!isZero(admittance_velocity_))
        {   
            navigation_velocity_.linear.x *= 0.0;
            navigation_velocity_.linear.y *= 0.0;
            navigation_velocity_.angular.z *= 0.0;

            RCLCPP_INFO(this->get_logger(), "Admittance Mode On");


        }

        publish_combined_velocity();
        
    }

    void publish_combined_velocity()
    {   
        RCLCPP_INFO(this->get_logger(), "publish_combined_velocity called");
        auto combined_velocity = geometry_msgs::msg::Twist();
        combined_velocity.linear.x = navigation_velocity_.linear.x + admittance_velocity_.linear.x;
        combined_velocity.linear.y = navigation_velocity_.linear.y + admittance_velocity_.linear.y;
        combined_velocity.angular.z = navigation_velocity_.angular.z + admittance_velocity_.angular.z;

        cmd_vel_pub_->publish(combined_velocity);
    }

private:


    bool isZero(const geometry_msgs::msg::Twist &twist) 
    {
        return (twist.linear.x >= -0.01 && twist.linear.x <= 0.01) &&
            (twist.linear.y >= -0.01 && twist.linear.y <= 0.01) &&
            (twist.linear.z >= -0.01 && twist.linear.z <= 0.01) &&
            (twist.angular.x >= -0.01 && twist.angular.x <= 0.01) &&
            (twist.angular.y >= -0.01 && twist.angular.y <= 0.01) &&
            (twist.angular.z >= -0.01 && twist.angular.z <= 0.01);
    }

    geometry_msgs::msg::Twist navigation_velocity_;
    geometry_msgs::msg::Twist admittance_velocity_;


    //Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_navigation;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_admittance;

    //Pubsliher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityManagerNode>());
  rclcpp::shutdown();
  return 0;
}

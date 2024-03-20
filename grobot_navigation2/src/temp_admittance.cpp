#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class AdmittanceInterface : public rclcpp::Node
{
public:
    AdmittanceInterface() : Node("admittance_interface")
    {
        subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, std::bind(&AdmittanceInterface::joint_states_callback, this, std::placeholders::_1));

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AdmittanceInterface::odom_callback, this, std::placeholders::_1));

        subscription_external_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/command_external", 10, std::bind(&AdmittanceInterface::external_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize parameters
        M_adm_ = Eigen::MatrixXd::Identity(3, 3);
        D_adm_ = Eigen::MatrixXd::Identity(3, 3);
        K_ = Eigen::MatrixXd::Identity(3, 3);

        r_vec_ = Eigen::VectorXd::Zero(3);
        r_dot_vec_ = Eigen::VectorXd::Zero(3);
        desired_r_vec_ = Eigen::VectorXd::Zero(3);
        desired_r_vel_ = Eigen::VectorXd::Zero(3);
        desired_r_acc_ = Eigen::VectorXd::Zero(3);

        tau_external_ = Eigen::VectorXd::Zero(3);
        tau_impedance_ = Eigen::VectorXd::Zero(3);

        x_tilde_ = Eigen::VectorXd::Zero(3);
        grad_V_imp_ = Eigen::VectorXd::Zero(3);
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr joint_states)
    {
        JointPosition[0] = joint_states->position[0];
        JointPosition[1] = joint_states->position[1];
        JointVelocity[0] = joint_states->velocity[0];
        JointVelocity[1] = joint_states->velocity[1];
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        RobotPosition[0] = odom->pose.pose.position.x;
        RobotPosition[1] = odom->pose.pose.position.y;
        RobotVelocity[0] = odom->twist.twist.linear.x;
        RobotVelocity[1] = odom->twist.twist.angular.z;
    }

    void external_callback(const std_msgs::msg::Float64MultiArray::SharedPtr external)
    {
        tau_external[0] = external->data[0];
        tau_external[1] = external->data[1];
        tau_external[2] = external->data[2];
    }

    void run()
    {
        rclcpp::Rate loop_rate(100);
        while (rclcpp::ok())
        {
            compute_impedance();
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = desired_r_vel_[0];
            cmd_vel_msg.linear.y = desired_r_vel_[1];
            cmd_vel_msg.angular.z = desired_r_vel_[2];
            cmd_vel_pub_->publish(cmd_vel_msg);

            rclcpp::spin_some(this->get_node_base_interface());
            loop_rate.sleep();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_external_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    Eigen::MatrixXd M_adm_;
    Eigen::MatrixXd D_adm_;
    Eigen::MatrixXd K_;

    Eigen::VectorXd r_vec_;
    Eigen::VectorXd r_dot_vec_;
    Eigen::VectorXd desired_r_vec_;
    Eigen::VectorXd desired_r_vel_;
    Eigen::VectorXd desired_r_acc_;

    Eigen::VectorXd tau_external_;
    Eigen::VectorXd tau_impedance_;

    Eigen::VectorXd x_tilde_;
    Eigen::VectorXd grad_V_imp_;

    void compute_impedance()
    {
        x_tilde_ = desired_r_vec_ - r_vec_;
        double V_imp = 0.5 * x_tilde_.transpose() * K_ * x_tilde_;
        grad_V_imp_ = K_ * x_tilde_;
        tau_impedance_ = -grad_V_imp_;
        desired_r_acc_ = M_adm_.inverse() * (tau_impedance_ + tau_external_ - D_adm_ * r_dot_vec_);
        desired_r_vel_ += desired_r_acc_ * dt;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdmittanceInterface>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

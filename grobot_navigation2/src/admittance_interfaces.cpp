#include "grobot_navigation2/admittance_interface.hpp"

void compute_impedance( const Eigen::MatrixXd& K,
                        Eigen::VectorXd& r_vec, Eigen::VectorXd& desired_r_vec,
                        Eigen::VectorXd& tau_impedance);

class AdmittanceInterface : public rclcpp::Node
{
    
public:
    AdmittanceInterface(): Node("admittance_interface")
    {   
        //Subscribers
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SystemDefaultsQoS(), std::bind(&AdmittanceInterface::joint_states_callback, this, std::placeholders::_1));

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SystemDefaultsQoS(), std::bind(&AdmittanceInterface::callback_odom, this, std::placeholders::_1));

        subscription_external_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/command_external", rclcpp::SystemDefaultsQoS(), std::bind(&AdmittanceInterface::callback_external, this, std::placeholders::_1));
        
        subscription_param_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/param_sub", rclcpp::SystemDefaultsQoS(), std::bind(&AdmittanceInterface::param_callback, this, std::placeholders::_1));

        // Publisher 
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_admittance", rclcpp::SystemDefaultsQoS());
        disturbance_ref_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("disturbance_ref",  rclcpp::SystemDefaultsQoS());
        
        //가상의 Mass(M_,M_ori_), Damping(D_), Stiffness(K_)
        double M_ = 1;
        double M_ori_ = 0.1;
        double D_ = 1;
        double K_ = 1;

        M_adm.diagonal() << M_, M_, M_;
        D_adm.diagonal() << D_,D_,D_;
        K.diagonal() << K_,K_,K_; 

    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr JointState_Data)
    {   

        //w
        JointPosition[0] = JointState_Data->position[0]; //left wheel
        JointPosition[1] = JointState_Data->position[1]; //right wheel

        //w_dot
        JointVelocity[0] = JointState_Data->velocity[0];
        JointVelocity[1] = JointState_Data->velocity[1];
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr  msg)
    {
        // Get r from the Odometry message
        RobotPosition[0] = msg->pose.pose.position.x;
        RobotPosition[1] = msg->pose.pose.position.y;
    
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RobotPosition[2] = yaw;

        // std::cout << "yaw: " <<RobotPosition[2] << std::endl;

        // Get \dot{r} from the Odometry message
        RobotVelocity[0] = msg->twist.twist.linear.x;
        RobotVelocity[1] = msg->twist.twist.angular.z;
    }

    void callback_external(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {   
        tau_external[0] = msg -> data[0];
        tau_external[1] = msg -> data[1];
        tau_external[2] = msg -> data[2];
        

    }

    void param_callback(const std_msgs::msg::Float64MultiArray::SharedPtr Param_Data)
    {
        M_adm.diagonal() <<  Param_Data -> data[0], Param_Data -> data[1], Param_Data -> data[2];
        D_adm.diagonal() <<  Param_Data -> data[3],  Param_Data -> data[4], Param_Data -> data[5];
        K.diagonal() << Param_Data -> data[6], Param_Data -> data[7], Param_Data -> data[8];

    }
    void run()  
    {   
        rclcpp::Rate loop_rate(100);
        while(rclcpp::ok())
        {       
            // RCLCPP_INFO(this->get_logger(), "루프 시작");
            //M,D,K값 실시간으로 바뀌는거 확인용
            // std::cout << "M: " << std::endl << M_adm << std::endl;
            // std::cout << "D: " << std::endl << D_adm << std::endl;
            // std::cout << "K: " << std::endl << K << std::endl;
        
            KDL::JntArray r(3);
            for (int i = 0; i < 3; i++) {
                r(i) = RobotPosition[i];
            }
            r_vec = r.data;

            KDL::JntArray r_dot(joint_size);
            for (int i = 0; i < joint_size; i++) {
                r_dot(i) = RobotVelocity[i];
            }
            r_dot_vec = r_dot.data;

            KDL::JntArray temp_external(3);
            for (int i = 0; i < 3; i++) {
                temp_external(i) = tau_external[i];
            }
            tau_external_ = temp_external.data;
            // desired_r_vec = tau_external_ ;
            
            // std::cout << "r_vec: " << std::endl << r_vec.transpose() << std::endl;
            // std::cout << "desired_r_vec: " << std::endl << desired_r_vec.transpose() << std::endl;


            //loop 도는데 걸리는 시간 측정
            dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
            last_update_time = rclcpp::Clock{}.now();

            //update desried_r 
            direction = tau_external_.normalized();
            magnitude = tau_external_.norm() * dt;
            desired_r_vec += direction * magnitude;

            compute_impedance(K,
                              r_vec, desired_r_vec,
                              tau_impedance);
            

            desired_r_acc = M_adm.inverse() * (tau_impedance + tau_external_ - D_adm * desired_r_vel);
            desired_r_vel += desired_r_acc * dt;
            
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = desired_r_vel[0];
            cmd_vel_msg.linear.y = desired_r_vel[1];
            cmd_vel_msg.angular.z = desired_r_vel[2];

            cmd_vel_pub->publish(cmd_vel_msg);

            //pub reference disturbance (plot 분석용)
            std_msgs::msg::Float64MultiArray msg_;
            msg_.data = {tau_external[0], tau_external[1], tau_external[2]};
            disturbance_ref_pub->publish(msg_);

            loop_rate.sleep();
        }
    }

    void compute_impedance( const Eigen::MatrixXd& K,
                            Eigen::VectorXd& r_vec, Eigen::VectorXd& desired_r_vec,
                            Eigen::VectorXd& tau_impedance)
    {   
        x_tilde = r_vec - desired_r_vec;

        V_imp = 0.5 * x_tilde.transpose() * K * x_tilde;

        grad_V_imp = K * x_tilde;
        tau_impedance = -grad_V_imp;
    }

private:
    int joint_size = 2;
    double dt = 0.0;
    rclcpp::Time last_update_time = rclcpp::Clock{}.now();

    Eigen::MatrixXd M_adm = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd D_adm = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3,3);

    Eigen::VectorXd r_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd r_dot_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desired_r_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desired_r_vel =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desired_r_acc =Eigen::VectorXd::Zero(3);

    Eigen::VectorXd tau_external_ =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd tau_impedance =Eigen::VectorXd::Zero(3);

    Eigen::VectorXd x_tilde =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd grad_V_imp =Eigen::VectorXd::Zero(3);

    Eigen::VectorXd direction =Eigen::VectorXd::Zero(3);

    float JointPosition[6] = {0.0};
    float JointVelocity[6] = {0.0};
    float RobotPosition[6] = {0.0};
    float RobotVelocity[6] = {0.0}; 
    float tau_external[6] = {0.0}; 
    
    double V_imp;
    double magnitude;

    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_external_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_param_;
    
    //Pubsliher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr disturbance_ref_pub;
    
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("Node ON"), "@@@@@@@@@@@ Admittance Interface node START @@@@@@@@@@@");

  auto admittance_interface = std::make_shared<AdmittanceInterface>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(admittance_interface);
  std::thread executor_thread([&executor]() { executor.spin(); });
  
  RCLCPP_INFO(admittance_interface->get_logger(), "노드 실행 준비 완료");
  admittance_interface->run();
  executor_thread.join();
  RCLCPP_INFO(admittance_interface->get_logger(), "노드 종료");
  rclcpp::shutdown();
  return 0;
}
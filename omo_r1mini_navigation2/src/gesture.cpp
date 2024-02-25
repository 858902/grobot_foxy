#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"


class GestureNode : public rclcpp::Node
{
public:
  GestureNode()
  : Node("gesture_node")
  {
    service_ = this->create_service<std_srvs::srv::SetBool>("gesture_service", 
      std::bind(&GestureNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    (void)request;  // 일단 요청을 사용 x 

    for (int i = 0; i < 30; ++i) {
      RCLCPP_INFO(this->get_logger(), "Gesture action %d", i + 1);
    } //동작 테스트 용

    response->success = true;
    response->message = "Successfully performed gesture action.";
  }

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GestureNode>());
  rclcpp::shutdown();
  return 0;
}

#include "mw_ahrs.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // char *port = "/dev/ttyUSB0";
    char *port = "/dev/ttyIMU";

    /*
        <ntrex::MwAhrsRosDriver>(port_name, baudrate, sel)

        sel 0 - 기능사용하지 않음
        sel 1 - Z축 캘리브레이션
        sel 2 - 각도리셋
        sel 3 - Z축 캘리브레이션및 각도리셋
    
    */
    rclcpp::spin(std::make_shared<ntrex::MwAhrsRosDriver>(port,115200,3));
    rclcpp::shutdown();
    return 0;
}

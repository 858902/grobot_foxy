/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
 
#include <grobot_motor/motor_node.hpp>

void LoadParameters(void)
{
  std::ifstream inFile("/home/ubuntu/ros2_ws/src/grobot_motor/data/motor_input.txt");
  if (!inFile.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Unable to open the file");
    return;
  }

  int i = 0;
  std::size_t found;
  for (std::string line; std::getline(inFile, line);)
  {
    found = line.find("=");

    switch (i)
    {
    case 0:
      pwm_range = atof(line.substr(found + 2).c_str());
      break;
    case 1:
      pwm_frequency = atof(line.substr(found + 2).c_str());
      break;
    case 2:
      pwm_limit = atof(line.substr(found + 2).c_str());
      break;
    case 3:
      control_cycle = atof(line.substr(found + 2).c_str());
      break;
    case 4:
      acceleration_ratio = atof(line.substr(found + 2).c_str());
      break;
    case 5:
      wheel_radius = atof(line.substr(found + 2).c_str());
      break;
    case 6:
      robot_radius = atof(line.substr(found + 2).c_str());
      break;
    case 7:
      encoder_resolution = atof(line.substr(found + 2).c_str());
      break;
      // case :  = atof(line.substr(found+2).c_str()); break;
    }
    i += 1;
  }
  inFile.close();
}

int InitMotors(void)
{
  pinum = pigpio_start(NULL, NULL);

  if (pinum < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Setup failed");
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_dir, PI_OUTPUT);
  set_mode(pinum, motor2_dir, PI_OUTPUT);
  set_mode(pinum, motor1_pwm, PI_OUTPUT);
  set_mode(pinum, motor2_pwm, PI_OUTPUT);
  set_mode(pinum, motor1_encA, PI_INPUT);
  set_mode(pinum, motor1_encB, PI_INPUT);
  set_mode(pinum, motor2_encA, PI_INPUT);
  set_mode(pinum, motor2_encB, PI_INPUT);

  gpio_write(pinum, motor1_dir, PI_LOW);
  gpio_write(pinum, motor2_dir, PI_LOW);

  set_PWM_range(pinum, motor1_pwm, pwm_range);
  set_PWM_range(pinum, motor2_pwm, pwm_range);
  set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
  set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);
  set_PWM_dutycycle(pinum, motor2_pwm, 0);

  set_pull_up_down(pinum, motor1_encA, PI_PUD_UP);
  set_pull_up_down(pinum, motor1_encB, PI_PUD_UP);
  set_pull_up_down(pinum, motor2_encA, PI_PUD_UP);
  set_pull_up_down(pinum, motor2_encB, PI_PUD_UP);

  current_pwm1 = 0;
  current_pwm2 = 0;

  current_direction1 = true;
  current_direction2 = true;

  acceleration = pwm_limit / (acceleration_ratio);

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
  return 0;
}

void SetInterrupts(void)
{
  callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
  callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
  callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
  callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1A--;
  else
    encoder_count_1A++;
  speed_count_1++;
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1B--;
  else
    encoder_count_1B++;
  speed_count_1++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2A--;
  else
    encoder_count_2A++;
  speed_count2++;
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2B--;
  else
    encoder_count_2B++;
  speed_count2++;
}

int SumMotor1Encoder()
{
  encoder_count_1 = encoder_count_1A + encoder_count_1B;
  return encoder_count_1;
}

int SumMotor2Encoder()
{
  encoder_count_2 = encoder_count_2A + encoder_count_2B;
  return encoder_count_2;
}

void InitEncoders(void)
{
  encoder_count_1 = 0;
  encoder_count_2 = 0;
  encoder_count_1A = 0;
  encoder_count_1B = 0;
  encoder_count_2A = 0;
  encoder_count_2B = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
  LoadParameters();
  InitMotors();
  InitEncoders();
  SetInterrupts();

  wheel_round = 2 * PI * wheel_radius;
  robot_round = 2 * PI * robot_radius;

  switch_direction = true;
  theta_distance_flag = 0;

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %d", acceleration_ratio);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

  printf("\033[2J");
}

void MotorController(int motor_num, bool direction, int pwm)
{
  int local_pwm = LimitPwm(pwm);

  if (motor_num == 1)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor1_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor1_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = false;
    }
  }

  else if (motor_num == 2)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor2_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor2_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = false;
    }
  }
}

void MoveMotor1_Distance(int pwm, double distance)
{
  double target_encoder = (encoder_resolution * 4 * distance) / wheel_round;
  int local_pwm = LimitPwm(pwm);
  bool direction = true;

  if (distance < 0)
  {
    direction = false;
    target_encoder = -target_encoder;
  }
  SumMotor1Encoder(); 

  // 목표 엔코더 카운트에 도달할 때까지 모터 제어
  if ((direction && encoder_count_1 < target_encoder) ||
      (!direction && encoder_count_1 > -target_encoder))
  {
    MotorController(1, direction, local_pwm);
  }

  else
  {
    // 목표 도달 후 모터 정지
    MotorController(1, direction, 0);
  }
  
}


int LimitPwm(int pwm)
{
  int output;
  if (pwm > pwm_limit * 2)
  {
    output = pwm_limit;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
  }
  else if (pwm > pwm_limit)
    output = pwm_limit;
  else if (pwm < 0)
  {
    output = 0;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
  }
  else
    output = pwm;
  return output;
}

void CalculateRpm()
{
  rpm_value1 = (speed_count_1 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count_1 = 0;
  rpm_value2 = (speed_count2 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count2 = 0;
}

void InfoMotors()
{
  CalculateRpm();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
  printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
  printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
  printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
  printf("enc2  :%11.0d\n", speed_count_1);
  
  printf("\n");
}

RosCommunicator::RosCommunicator()
    : Node("grobot_motor"), count_(0)
{
  timer_ = this->create_wall_timer(
      100ms, std::bind(&RosCommunicator::TimerCallback, this));

  subscription_cart = this->create_subscription<std_msgs::msg::String>(
        "integrate_cart", 10, std::bind(&RosCommunicator::cart_callback, this, std::placeholders::_1));
}

void RosCommunicator::TimerCallback()
{  
  // MotorController(1, true, 100);
  // MotorController(2, true, 100);
  // MoveMotor1_Distance(110,0.1);
  // MoveMotor1_Distance(110,-0.1);
  if(start_signal_) 
  {
    MoveMotor1_Distance(110, -0.1);
  }

  else if(finish_signal_)
  {
    MoveMotor1_Distance(110, 0.0); 
  }

  InfoMotors();
}

void RosCommunicator::cart_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "start") 
  {   
      start_signal_ = true;
      finish_signal_ = false;

  }

  if(msg->data == "finish") 
  {   
      start_signal_ = false;
      finish_signal_ = true;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  MotorController(1, true, 0);
  MotorController(2, true, 0);
  return 0;
}

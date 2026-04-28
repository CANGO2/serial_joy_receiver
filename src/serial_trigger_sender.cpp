#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "cango_msgs/msg/robot_control.hpp"

#include <cstring>
#include <string>
#include <algorithm>

class SerialTriggerSender : public rclcpp::Node
{
public:
  SerialTriggerSender()
  : Node("serial_trigger_sender")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 115200);

    // master에서 오는 진동 명령만 받음
    declare_parameter<std::string>("control_topic", "/master2control");

    declare_parameter<int>("vibration_pwm_on", 120);
    declare_parameter<int>("vibration_pwm_off", 0);

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    control_topic_ = get_parameter("control_topic").as_string();

    if (!open_serial()) {
      throw std::runtime_error("Failed to open serial");
    }

    sub_ = create_subscription<cango_msgs::msg::RobotControl>(
      control_topic_,
      10,
      std::bind(&SerialTriggerSender::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      get_logger(),
      "Vibration sender ready. sub topic: %s, port: %s",
      control_topic_.c_str(),
      port_.c_str()
    );
  }

private:
  int fd_ = -1;

  std::string port_;
  std::string control_topic_;
  int baudrate_;

  bool last_vibration_ = false;
  bool has_last_vibration_ = false;

  rclcpp::Subscription<cango_msgs::msg::RobotControl>::SharedPtr sub_;

  bool open_serial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);

    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open failed: %s", strerror(errno));
      return false;
    }

    termios tty {};

    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed;

    switch (baudrate_) {
      case 9600:
        speed = B9600;
        break;

      case 57600:
        speed = B57600;
        break;

      case 115200:
        speed = B115200;
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unsupported baudrate: %d", baudrate_);
        close(fd_);
        fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    return true;
  }

  uint8_t checksum(uint8_t type, uint8_t len, uint8_t data)
  {
    return type ^ len ^ data;
  }

  void send_packet(uint8_t pwm)
  {
    if (fd_ < 0) {
      return;
    }

    uint8_t packet[6];

    packet[0] = 0xFF;
    packet[1] = 0x55;
    packet[2] = 0x02;
    packet[3] = 0x01;
    packet[4] = pwm;
    packet[5] = checksum(packet[2], packet[3], packet[4]);

    ssize_t written = write(fd_, packet, 6);

    if (written != 6) {
      RCLCPP_WARN(
        get_logger(),
        "Serial write incomplete. written: %ld",
        written
      );
      return;
    }

    RCLCPP_INFO(get_logger(), "Vibration PWM Sent: %d", pwm);
  }

  void callback(const cango_msgs::msg::RobotControl::SharedPtr msg)
  {
    // master2control에서 vibration만 사용
    bool vibration = msg->vibration;

    // 값이 바뀔 때만 전송
    if (has_last_vibration_ && vibration == last_vibration_) {
      return;
    }

    has_last_vibration_ = true;
    last_vibration_ = vibration;

    int pwm_on = get_parameter("vibration_pwm_on").as_int();
    int pwm_off = get_parameter("vibration_pwm_off").as_int();

    int pwm = vibration ? pwm_on : pwm_off;
    pwm = std::clamp(pwm, 0, 255);

    send_packet(static_cast<uint8_t>(pwm));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialTriggerSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
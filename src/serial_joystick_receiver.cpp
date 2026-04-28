#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class SerialJoystickReceiver : public rclcpp::Node
{
public:
  SerialJoystickReceiver()
  : Node("serial_joystick_receiver"),
    fd_(-1),
    state_(ParseState::WAIT_HEADER1),
    type_(0),
    length_(0),
    data_index_(0)
  {
 this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<double>("center_x", 512.0);
    this->declare_parameter<double>("center_y", 512.0);
    this->declare_parameter<double>("scale_x", 512.0);
    this->declare_parameter<double>("scale_y", 512.0);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    scale_x_ = this->get_parameter("scale_x").as_double();
    scale_y_ = this->get_parameter("scale_y").as_double();

    raw_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/joystick_raw", 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    if (!open_serial()) {
      throw std::runtime_error("Failed to open serial port");
    }

    timer_ = this->create_wall_timer(2ms, std::bind(&SerialJoystickReceiver::read_serial, this));

    RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d", port_.c_str(), baudrate_);
  }

  ~SerialJoystickReceiver() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  static constexpr uint8_t HEADER1 = 0xFF;
  static constexpr uint8_t HEADER2 = 0x55;
  static constexpr uint8_t PKT_JOYSTICK = 0x01;
  static constexpr uint8_t MAX_DATA_LEN = 32;

  enum class ParseState
  {
    WAIT_HEADER1,
    WAIT_HEADER2,
    READ_TYPE,
    READ_LENGTH,
    READ_DATA,
    READ_CHECKSUM
  };

  bool open_serial()
  {
    fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed;
    switch (baudrate_) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d", baudrate_);
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

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    tcflush(fd_, TCIFLUSH);
    return true;
  }

  uint8_t calc_checksum(uint8_t type, uint8_t length, const std::array<uint8_t, MAX_DATA_LEN> & data)
  {
    uint8_t cs = 0;
    cs ^= type;
    cs ^= length;
    for (size_t i = 0; i < length; ++i) {
      cs ^= data[i];
    }
    return cs;
  }

  float normalize_axis(int value, double center, double scale)
  {
    double norm = (static_cast<double>(value) - center) / scale;
    if (norm > 1.0) norm = 1.0;
    if (norm < -1.0) norm = -1.0;
    return static_cast<float>(norm);
  }

  void publish_packet(uint16_t x, uint16_t y, uint8_t button)
  {
    std_msgs::msg::Int32MultiArray raw_msg;
    raw_msg.data = {
      static_cast<int32_t>(x),
      static_cast<int32_t>(y),
      static_cast<int32_t>(button)
    };
    raw_pub_->publish(raw_msg);

    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.axes = {
      normalize_axis(static_cast<int>(x), center_x_, scale_x_),
      normalize_axis(static_cast<int>(y), center_y_, scale_y_)
    };
    joy_msg.buttons = {static_cast<int32_t>(button)};
    joy_pub_->publish(joy_msg);
  }

  void handle_complete_packet(uint8_t recv_checksum)
  {
    uint8_t expected = calc_checksum(type_, length_, data_);
    if (recv_checksum != expected) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Checksum mismatch");
      reset_parser();
      return;
    }

    if (type_ == PKT_JOYSTICK && length_ == 5) {
      uint16_t x = static_cast<uint16_t>(data_[0] | (data_[1] << 8));
      uint16_t y = static_cast<uint16_t>(data_[2] | (data_[3] << 8));
      uint8_t button = data_[4];
      publish_packet(x, y, button);
    }

    reset_parser();
  }

  void reset_parser()
  {
    state_ = ParseState::WAIT_HEADER1;
    type_ = 0;
    length_ = 0;
    data_index_ = 0;
  }

  void parse_byte(uint8_t b)
  {
    switch (state_) {
      case ParseState::WAIT_HEADER1:
        if (b == HEADER1) {
          state_ = ParseState::WAIT_HEADER2;
        }
        break;

      case ParseState::WAIT_HEADER2:
        if (b == HEADER2) {
          state_ = ParseState::READ_TYPE;
        } else if (b == HEADER1) {
          state_ = ParseState::WAIT_HEADER2;
        } else {
          reset_parser();
        }
        break;

      case ParseState::READ_TYPE:
        type_ = b;
        state_ = ParseState::READ_LENGTH;
        break;

      case ParseState::READ_LENGTH:
        length_ = b;
        if (length_ == 0 || length_ > MAX_DATA_LEN) {
          reset_parser();
          break;
        }
        if (type_ == PKT_JOYSTICK && length_ == 5) {
          data_index_ = 0;
          state_ = ParseState::READ_DATA;
        } else {
          reset_parser();
        }
        break;

      case ParseState::READ_DATA:
        data_[data_index_++] = b;
        if (data_index_ >= length_) {
          state_ = ParseState::READ_CHECKSUM;
        }
        break;

      case ParseState::READ_CHECKSUM:
        handle_complete_packet(b);
        break;
    }
  }

  void read_serial()
  {
    if (fd_ < 0) {
      return;
    }

    uint8_t buffer[256];
    ssize_t n = read(fd_, buffer, sizeof(buffer));

    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Serial read error: %s", std::strerror(errno));
      }
      return;
    }

    if (n == 0) {
      return;
    }

    for (ssize_t i = 0; i < n; ++i) {
      parse_byte(buffer[i]);
    }
  }

  std::string port_;
  int baudrate_;
  double center_x_;
  double center_y_;
  double scale_x_;
  double scale_y_;

  int fd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

  ParseState state_;
  uint8_t type_;
  uint8_t length_;
  std::array<uint8_t, MAX_DATA_LEN> data_{};
  uint8_t data_index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SerialJoystickReceiver>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "Fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

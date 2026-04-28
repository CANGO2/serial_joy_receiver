#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "cango_msgs/msg/robot_control.hpp"

#include <algorithm>
#include <cmath>
#include <string>

class JoyToRobotControl : public rclcpp::Node
{
public:
  JoyToRobotControl() : Node("joy_to_robot_control")
  {
    declare_parameter<double>("max_linear_speed", 0.3);
    declare_parameter<double>("max_ang_speed", 0.8);
    declare_parameter<double>("max_side_speed", 0.2);
    declare_parameter<double>("deadzone", 0.08);
    declare_parameter<double>("button_hold_time_sec", 2.0);

    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>("control_out_topic", "/cango_control_out");
    declare_parameter<std::string>("control_final_topic", "/cango_control_final");

    declare_parameter<std::string>("master_sub_topic", "/master2control");

    joy_topic_ = get_parameter("joy_topic").as_string();
    control_out_topic_ = get_parameter("control_out_topic").as_string();
    control_final_topic_ = get_parameter("control_final_topic").as_string();
    master_sub_topic_ = get_parameter("master_sub_topic").as_string();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      std::bind(&JoyToRobotControl::joyCallback, this, std::placeholders::_1)
    );

    mode_feedback_sub_ = create_subscription<cango_msgs::msg::RobotControl>(
      control_final_topic_, 10,
      std::bind(&JoyToRobotControl::modeFeedbackCallback, this, std::placeholders::_1)
    );

    master_sub_ = create_subscription<cango_msgs::msg::RobotControl>(
      master_sub_topic_, 10,
      std::bind(&JoyToRobotControl::masterCallback, this, std::placeholders::_1)
    );

    control_pub_ = create_publisher<cango_msgs::msg::RobotControl>(
      control_out_topic_, 10
    );

    RCLCPP_INFO(
      get_logger(),
      "JoyToRobotControl started | pub: %s | final sub: %s | master sub: %s",
      control_out_topic_.c_str(),
      control_final_topic_.c_str(),
      master_sub_topic_.c_str()
    );
  }

private:
  double applyDeadzone(double value)
  {
    double deadzone = get_parameter("deadzone").as_double();

    if (std::abs(value) < deadzone) {
      return 0.0;
    }

    return std::clamp(value, -1.0, 1.0);
  }

  void modeFeedbackCallback(const cango_msgs::msg::RobotControl::SharedPtr msg)
  {
    current_mode_ = msg->mode;
    vibration_state_ = msg->vibration;
    has_mode_feedback_ = true;
  }

  void masterCallback(const cango_msgs::msg::RobotControl::SharedPtr msg)
  {
    vibration_state_ = msg->vibration;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "master2control received | vibration: %d",
      vibration_state_
    );
  }

  void setCommonFields(cango_msgs::msg::RobotControl & cmd)
  {
    cmd.robot_angle = 0.0f;

    // 여기서 robot_up을 false로 강제하면 안 됨
    // robot_up은 control_logic_node에서 최종 결정
    cmd.robot_up = false;

    cmd.vibration = vibration_state_;
  }

  void publishStopMode0(bool button_pressed)
  {
    cango_msgs::msg::RobotControl cmd;

    cmd.mode = 0.0f;
    cmd.linear_speed = 0.0f;
    cmd.side_speed = 0.0f;
    cmd.ang_speed = 0.0f;

    setCommonFields(cmd);

    // 여기서는 /cango_control_out만 publish
    control_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "mode: 0.0 | joystick blocked | button: %d | hold_active: %d | vibration: %d",
      button_pressed,
      button_hold_active_,
      vibration_state_
    );
  }

  void publishJoystickMode1(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    bool button_pressed)
  {
    double x = -applyDeadzone(msg->axes[0]);
    double y = -applyDeadzone(msg->axes[1]);

    double max_linear = get_parameter("max_linear_speed").as_double();
    double max_ang = get_parameter("max_ang_speed").as_double();
    double max_side = get_parameter("max_side_speed").as_double();

    cango_msgs::msg::RobotControl cmd;

    cmd.mode = 1.0f;
    cmd.linear_speed = static_cast<float>(y * max_linear);

    if (button_pressed) {
      cmd.side_speed = static_cast<float>(x * max_side);
      cmd.ang_speed = 0.0f;
    } else {
      cmd.side_speed = 0.0f;
      cmd.ang_speed = static_cast<float>(x * max_ang);
    }

    setCommonFields(cmd);

    // 여기서도 /cango_control_out만 publish
    control_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      300,
      "mode: 1.0 | linear: %.2f | side: %.2f | ang: %.2f | vibration: %d",
      cmd.linear_speed,
      cmd.side_speed,
      cmd.ang_speed,
      cmd.vibration
    );
  }

  void requestMode1()
  {
    current_mode_ = 1.0f;
    button_hold_active_ = false;

    cango_msgs::msg::RobotControl cmd;

    cmd.mode = 1.0f;
    cmd.linear_speed = 0.0f;
    cmd.side_speed = 0.0f;
    cmd.ang_speed = 0.0f;

    setCommonFields(cmd);

    control_pub_->publish(cmd);

    RCLCPP_WARN(get_logger(), "Button held. Request mode change to 1.");
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 2) {
      return;
    }

    bool button_pressed = false;
    if (!msg->buttons.empty()) {
      button_pressed = (msg->buttons[0] == 1);
    }

    if (std::abs(current_mode_ - 0.0f) < 0.001f)
    {
      if (button_pressed)
      {
        if (!button_hold_active_)
        {
          button_hold_active_ = true;
          button_hold_start_time_ = now();
        }
        else
        {
          double hold_time = get_parameter("button_hold_time_sec").as_double();
          double elapsed = (now() - button_hold_start_time_).seconds();

          if (elapsed >= hold_time) {
            requestMode1();
            return;
          }
        }
      }
      else
      {
        button_hold_active_ = false;
      }

      publishStopMode0(button_pressed);
      return;
    }

    if (std::abs(current_mode_ - 1.0f) < 0.001f)
    {
      button_hold_active_ = false;
      publishJoystickMode1(msg, button_pressed);
      return;
    }

    publishStopMode0(button_pressed);
  }

  std::string joy_topic_;
  std::string control_out_topic_;
  std::string control_final_topic_;
  std::string master_sub_topic_;

  float current_mode_ = 0.0f;
  bool has_mode_feedback_ = false;
  bool vibration_state_ = false;

  bool button_hold_active_ = false;
  rclcpp::Time button_hold_start_time_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<cango_msgs::msg::RobotControl>::SharedPtr mode_feedback_sub_;
  rclcpp::Subscription<cango_msgs::msg::RobotControl>::SharedPtr master_sub_;

  rclcpp::Publisher<cango_msgs::msg::RobotControl>::SharedPtr control_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToRobotControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
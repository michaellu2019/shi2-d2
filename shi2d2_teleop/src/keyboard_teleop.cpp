#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

#include "shi2d2_teleop/teleop.hpp"

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop() : Node("keyboard_teleop")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("teleop_commands", TELEOP_LOOP_PERIOD_MS);
    timer_ = this->create_wall_timer(TELEOP_LOOP_PERIOD_MS * 1ms, std::bind(&KeyboardTeleop::timer_callback, this));
    configure_terminal();
  }

  ~KeyboardTeleop()
  {
    reset_terminal();
  }

private:
  void configure_terminal()
  {
    // Save the current terminal settings
    tcgetattr(STDIN_FILENO, &original_terminal_settings_);
    termios new_settings = original_terminal_settings_;

    // Disable canonical mode and echo
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    // // Set the terminal to non-blocking mode
    // int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    // fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  void reset_terminal()
  {
    // Restore the original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_settings_);
  }

  void timer_callback()
  {
    char input_char;
    auto message = std_msgs::msg::Int8();
    message.data = STOP;
    if (read(STDIN_FILENO, &input_char, 1) == 1) {
      switch (input_char) {
        case 'w':
          message.data = FORWARD;
          break;
        case 'a':
          message.data = LEFT;
          break;
        case 's':
          message.data = BACKWARD;
          break;
        case 'd':
          message.data = RIGHT;
          break;
        case 'q':
          message.data = COUNTERCLOCKWISE;
          break;
        case 'e':
          message.data = CLOCKWISE;
          break;
        default:
          message.data = STOP;
          break;
      }

      RCLCPP_INFO(this->get_logger(), "Publishing: '%d' for '%c", message.data, input_char);
    }

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  termios original_terminal_settings_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
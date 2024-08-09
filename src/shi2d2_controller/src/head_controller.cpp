#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class HeadController : public rclcpp::Node
{
  public:
    HeadController() : Node("head_controller"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("head_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&HeadController::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = trajectory_msgs::msg::JointTrajectory();

      message.joint_names.push_back("head_upper_neck_joint");
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      double head_upper_neck_joint_angle = 1.57 * sin(count_ * 1.0);
      // double head_upper_neck_joint_angle = 0.0;
      point.positions.push_back(head_upper_neck_joint_angle);
      point.time_from_start.sec = 1.0;
      point.time_from_start.nanosec = 0.0;
      message.points.push_back(point);
      publisher_->publish(message);

      count_++;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", head_upper_neck_joint_angle);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadController>());
  rclcpp::shutdown();
  return 0;
}

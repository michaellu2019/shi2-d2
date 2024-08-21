#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class LeftLegController : public rclcpp::Node
{
  public:
    LeftLegController() : Node("left_leg_controller"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("left_leg_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&LeftLegController::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = trajectory_msgs::msg::JointTrajectory();

      message.joint_names.push_back("left_upper_hip_body_joint");
      message.joint_names.push_back("left_lower_hip_left_upper_hip_joint");
      message.joint_names.push_back("left_upper_leg_left_lower_hip_joint");
      message.joint_names.push_back("left_lower_leg_left_upper_leg_joint");
      message.joint_names.push_back("left_foot_left_lower_leg_joint");

      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      double leg_joint_angle = 3.14/4 * sin(count_ * 0.5);
      point.positions.push_back(leg_joint_angle);
      point.positions.push_back(leg_joint_angle);
      point.positions.push_back(leg_joint_angle);
      point.positions.push_back(leg_joint_angle);
      point.positions.push_back(leg_joint_angle);
      point.time_from_start.sec = 1.0;
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
  rclcpp::spin(std::make_shared<LeftLegController>());
  rclcpp::shutdown();
  return 0;
}

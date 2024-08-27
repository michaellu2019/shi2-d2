#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
  public:
    LegController() : Node("leg_controller"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("leg_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&LegController::timer_callback, this));
    }

  private:
    void add_joint_position(trajectory_msgs::msg::JointTrajectory::SharedPtr traj, trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr point,
                            const std::string& joint_name, double joint_angle)
    {
      traj->joint_names.push_back(joint_name);
      point->positions.push_back(joint_angle);
    }

    void timer_callback()
    {
      auto message = trajectory_msgs::msg::JointTrajectory();
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      auto joint_traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      auto joint_traj_point = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();

      double left_leg_joint_angle = 3.14/4;// * sin(count_ * 0.5);
      double right_leg_joint_angle = -3.14/4;// * sin(count_ * 0.5);

      // message.joint_names.push_back("left_upper_hip_body_joint");
      // point.positions.push_back(left_leg_joint_angle);
      // message.joint_names.push_back("left_lower_hip_left_upper_hip_joint");
      // point.positions.push_back(left_leg_joint_angle);
      // message.joint_names.push_back("left_upper_leg_left_lower_hip_joint");
      // point.positions.push_back(left_leg_joint_angle);
      // message.joint_names.push_back("left_lower_leg_left_upper_leg_joint");
      // point.positions.push_back(left_leg_joint_angle);
      // message.joint_names.push_back("left_foot_left_lower_leg_joint");
      // point.positions.push_back(left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_upper_hip_body_joint", left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_lower_hip_left_upper_hip_joint", left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_upper_leg_left_lower_hip_joint", left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_lower_leg_left_upper_leg_joint", left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_foot_left_lower_leg_joint", left_leg_joint_angle);

      // message.joint_names.push_back("right_upper_hip_body_joint");
      // point.positions.push_back(right_leg_joint_angle);
      // message.joint_names.push_back("right_lower_hip_right_upper_hip_joint");
      // point.positions.push_back(right_leg_joint_angle);
      // message.joint_names.push_back("right_upper_leg_right_lower_hip_joint");
      // point.positions.push_back(right_leg_joint_angle);
      // message.joint_names.push_back("right_lower_leg_right_upper_leg_joint");
      // point.positions.push_back(right_leg_joint_angle);
      // message.joint_names.push_back("right_foot_right_lower_leg_joint");
      // point.positions.push_back(right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_upper_hip_body_joint", right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_lower_hip_right_upper_hip_joint", right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_upper_leg_right_lower_hip_joint", right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_lower_leg_right_upper_leg_joint", right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_foot_right_lower_leg_joint", right_leg_joint_angle);
      
      // point.time_from_start.sec = 1.0;
      // message.points.push_back(point);
      // publisher_->publish(message);

      joint_traj_point->time_from_start.sec = 1.0;
      joint_traj->points.push_back(*joint_traj_point);
      publisher_->publish(*joint_traj);

      count_++;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' and '%f", left_leg_joint_angle, right_leg_joint_angle);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}

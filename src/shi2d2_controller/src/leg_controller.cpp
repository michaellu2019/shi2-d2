#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
  public:
    LegController() : Node("leg_controller"), count_(0)
    {
      joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("leg_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(100ms, std::bind(&LegController::timer_callback, this));

      start_time_ = rclcpp::Time();  // Initialize start time
      clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10,
        [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_callback(msg);
        });
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

      double left_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 2.0 * M_PI) - M_PI/4.0;
      double right_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 2.0 * M_PI) - M_PI/4.0;
      add_joint_position(joint_traj, joint_traj_point, "left_upper_hip_body_joint", 0.0);
      add_joint_position(joint_traj, joint_traj_point, "left_lower_hip_left_upper_hip_joint", 0.0);
      add_joint_position(joint_traj, joint_traj_point, "left_upper_leg_left_lower_hip_joint", -0.5 * left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_lower_leg_left_upper_leg_joint", left_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "left_foot_left_lower_leg_joint", -0.5 * left_leg_joint_angle);
      
      add_joint_position(joint_traj, joint_traj_point, "right_upper_hip_body_joint", 0.0);
      add_joint_position(joint_traj, joint_traj_point, "right_lower_hip_right_upper_hip_joint", 0.0);
      add_joint_position(joint_traj, joint_traj_point, "right_upper_leg_right_lower_hip_joint", 0.5 * right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_lower_leg_right_upper_leg_joint", right_leg_joint_angle);
      add_joint_position(joint_traj, joint_traj_point, "right_foot_right_lower_leg_joint", -0.5 * right_leg_joint_angle);
      
      joint_traj_point->time_from_start.nanosec = 0.01e9;
      joint_traj->points.push_back(*joint_traj_point);
      joint_traj_publisher_->publish(*joint_traj);

      count_++;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' and '%f", left_leg_joint_angle, right_leg_joint_angle);
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
      if (start_time_.nanoseconds() == 0) {
          start_time_ = msg->clock;
      }
      sim_time_ = msg->clock;
      
      if (start_time_.nanoseconds() != 0) {
        auto sim_time_elapsed_ = sim_time_ - start_time_;
        sim_time_elapsed_sec_ = sim_time_elapsed_.seconds();
        RCLCPP_INFO(this->get_logger(), "Time since simulation start: %.2fs...", sim_time_elapsed_sec_);
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_publisher_;
    size_t count_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Time start_time_;
    rclcpp::Time sim_time_;
    double sim_time_elapsed_sec_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}

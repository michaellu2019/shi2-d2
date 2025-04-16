#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "shi2d2_controller/shi2d2_controller.hpp"

using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
  public:
    LegController() : Node("leg_controller"), count_(0)
    {
      left_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("left_leg_group_controller/joint_trajectory", 10);
      right_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("right_leg_group_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&LegController::timer_callback, this));

      start_time_ = rclcpp::Time();
      clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10,
        [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_callback(msg);
        });

      for (int i = 0; i < NUM_LEG_JOINTS; i++) {
        std::cout << i << ": " << LEFT_LEG_JOINTS[i] << " and " << RIGHT_LEG_JOINTS[i] << std::endl;
        left_leg_joint_traj_.joint_names.push_back(LEFT_LEG_JOINTS[i]);
        left_leg_joint_traj_point_.positions.push_back(0.0);
        right_leg_joint_traj_.joint_names.push_back(RIGHT_LEG_JOINTS[i]);
        right_leg_joint_traj_point_.positions.push_back(0.0);
      }
      
      RCLCPP_INFO(this->get_logger(), "FINISHED INIT...");
    }

  private:
    void solve_leg_ik(int leg_id, FootPose foot_pose, LegJointAngles &leg_joint_angles)
    {

      std::cout << " X: " << foot_pose.position.x
                << " Y: " << foot_pose.position.y
                << " Z: " << foot_pose.position.z
                << " YAW: " << foot_pose.rotation.yaw
                << std::endl;

      double x = foot_pose.position.x + BODY_TO_FOOT_DISTANCE_MM.x;
      double y = -foot_pose.position.y + BODY_TO_FOOT_DISTANCE_MM.y;
      double z = -foot_pose.position.z + BODY_TO_FOOT_DISTANCE_MM.z - UPPER_HIP_LINK_LENGTH_MM - FOOT_LINK_LENGTH_MM;
      double pitch = foot_pose.rotation.pitch;
      double yaw = foot_pose.rotation.yaw;

      std::cout << " TX: " << x
                << " TY: " << y
                << " TZ: " << z
                << " TYAW: " << yaw
                << std::endl;

      double knee_bend_angle_num = x*x + z*z - UPPER_LEG_LINK_LENGTH_MM*UPPER_LEG_LINK_LENGTH_MM - LOWER_HIP_LINK_LENGTH_MM*LOWER_HIP_LINK_LENGTH_MM;
      double knee_bend_angle_den = -2 * UPPER_LEG_LINK_LENGTH_MM * LOWER_LEG_LINK_LENGTH_MM;
      double knee_bend_angle = acos(knee_bend_angle_num/knee_bend_angle_den);

      std::cout << "NUM: " << knee_bend_angle_num << " DEN: " << knee_bend_angle_den << std::endl;
      std::cout << " KNEE BEND: " << (knee_bend_angle * 180.0/M_PI) << std::endl;

      leg_joint_angles.lower_leg_upper_leg_joint_angle = -(M_PI - knee_bend_angle);
      leg_joint_angles.upper_leg_lower_hip_joint_angle = atan2(x, z) + asin(LOWER_LEG_LINK_LENGTH_MM * sin(knee_bend_angle)/sqrt(x*x + z*z));
      leg_joint_angles.foot_lower_leg_joint_angle = -leg_joint_angles.upper_leg_lower_hip_joint_angle - leg_joint_angles.lower_leg_upper_leg_joint_angle + pitch;
      
      double hip_roll_angle1 = acos(LOWER_HIP_LINK_LENGTH_MM/sqrt(y*y + z*z));
      double hip_roll_angle2 = acos(z/sqrt(y*y + z*z));
      
      leg_joint_angles.lower_hip_upper_hip_joint_angle = hip_roll_angle1 + hip_roll_angle2 - M_PI/2.0;

      std::cout << " HIP2: " << (leg_joint_angles.lower_hip_upper_hip_joint_angle * 180.0/M_PI)
                << " HIP3: " << (leg_joint_angles.upper_leg_lower_hip_joint_angle * 180.0/M_PI)
                << " KNEE: " << (leg_joint_angles.lower_leg_upper_leg_joint_angle * 180.0/M_PI) 
                << " ANKLE: " << (leg_joint_angles.foot_lower_leg_joint_angle * 180.0/M_PI)
                << std::endl;
    }

    void set_leg_joint_angles(int leg_id, LegJointAngles &leg_joint_angles)
    {
      set_leg_joint_angle(leg_id, UPPER_HIP_BODY_JOINT, leg_joint_angles.upper_hip_body_joint_angle);
      set_leg_joint_angle(leg_id, LOWER_HIP_UPPER_HIP_JOINT, leg_joint_angles.lower_hip_upper_hip_joint_angle);
      set_leg_joint_angle(leg_id, UPPER_LEG_LOWER_HIP_JOINT, leg_joint_angles.upper_leg_lower_hip_joint_angle);
      set_leg_joint_angle(leg_id, LOWER_LEG_UPPER_LEG_JOINT, leg_joint_angles.lower_leg_upper_leg_joint_angle);
      set_leg_joint_angle(leg_id, FOOT_LOWER_LEG_JOINT, leg_joint_angles.foot_lower_leg_joint_angle);
    }

    void set_leg_joint_angle(int leg_id, int joint_id, double angle) {
      if (leg_id == LEFT_LEG) {
        left_leg_joint_traj_point_.positions[joint_id] = angle;
      } else if (leg_id == RIGHT_LEG) {
        right_leg_joint_traj_point_.positions[joint_id] = angle;
      }
    }

    void write_leg_angles()
    {
      left_leg_joint_traj_point_.time_from_start.nanosec = TRAJECTORY_TIME_FROM_START_NS;
      left_leg_joint_traj_.points.push_back(left_leg_joint_traj_point_);
      left_leg_joint_traj_publisher_->publish(left_leg_joint_traj_);
      left_leg_joint_traj_.points.clear();

      right_leg_joint_traj_point_.time_from_start.nanosec = TRAJECTORY_TIME_FROM_START_NS;
      right_leg_joint_traj_.points.push_back(right_leg_joint_traj_point_);
      right_leg_joint_traj_publisher_->publish(right_leg_joint_traj_);
      right_leg_joint_traj_.points.clear();
    }

    void timer_callback()
    {
      // double left_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 1.5 * M_PI) - M_PI/4.0;
      // double right_leg_joint_angle = M_PI/4.0 * cos(sim_time_elapsed_sec_ * 1.5 * M_PI) - M_PI/4.0;
      // // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' and '%f", left_leg_joint_angle, right_leg_joint_angle);

      // set_leg_joint_angle(LEFT_LEG, LEFT_UPPER_LEG_LEFT_LOWER_HIP_JOINT, -0.5 * left_leg_joint_angle);
      // set_leg_joint_angle(LEFT_LEG, LEFT_LOWER_LEG_LEFT_UPPER_LEG_JOINT, left_leg_joint_angle);
      // set_leg_joint_angle(LEFT_LEG, LEFT_FOOT_LEFT_LOWER_LEG_JOINT, -0.5 * left_leg_joint_angle);

      // set_leg_joint_angle(RIGHT_LEG, RIGHT_UPPER_LEG_RIGHT_LOWER_HIP_JOINT, 0.5 * right_leg_joint_angle);
      // set_leg_joint_angle(RIGHT_LEG, RIGHT_LOWER_LEG_RIGHT_UPPER_LEG_JOINT, right_leg_joint_angle);
      // set_leg_joint_angle(RIGHT_LEG, RIGHT_FOOT_RIGHT_LOWER_LEG_JOINT, -0.5 * right_leg_joint_angle);
      double fx = ((int) count_/200) % 2 == 0 ? 40.0 : 0.0;
      double fy = ((int) count_/200) % 2 == 0 ? 20.0 : 0.0;
      double fz = ((int) count_/200) % 2 == 0 ? 20.0 : 60.0;

      Position foot_position = {fx, fy, fz};
      Rotation foot_rotation = {0.0, 0.0, 0.0};
      FootPose foot_pose = {foot_position, foot_rotation};

      LegJointAngles left_leg_joint_angles;
      solve_leg_ik(LEFT_LEG, foot_pose, left_leg_joint_angles);
      set_leg_joint_angles(LEFT_LEG, left_leg_joint_angles);

      double leg_joint_angle = ((int) count_++/100) % 2 == 0 ? 0.0 : M_PI/4.0;
      // set_leg_joint_angle(LEFT_LEG, LEFT_UPPER_LEG_LEFT_LOWER_HIP_JOINT, leg_joint_angle);
      // set_leg_joint_angle(LEFT_LEG, LEFT_LOWER_LEG_LEFT_UPPER_LEG_JOINT, leg_joint_angle);
      // set_leg_joint_angle(LEFT_LEG, LEFT_FOOT_LEFT_LOWER_LEG_JOINT, leg_joint_angle);

      // set_leg_joint_angle(RIGHT_LEG, RIGHT_FOOT_RIGHT_LOWER_LEG_JOINT, leg_joint_angle);


      write_leg_angles();

      count_++;
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
        // RCLCPP_INFO(this->get_logger(), "Time since simulation start: %.2fs...", sim_time_elapsed_sec_);
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_leg_joint_traj_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_leg_joint_traj_publisher_;

    trajectory_msgs::msg::JointTrajectory left_leg_joint_traj_ = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint left_leg_joint_traj_point_ = trajectory_msgs::msg::JointTrajectoryPoint();
    trajectory_msgs::msg::JointTrajectory right_leg_joint_traj_ = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint right_leg_joint_traj_point_ = trajectory_msgs::msg::JointTrajectoryPoint();

    size_t count_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Time start_time_;
    rclcpp::Time sim_time_;
    double sim_time_elapsed_sec_;

    std::map<std::string, double> left_leg_controller_joint_angles_;
    std::map<std::string, double> right_leg_controller_joint_angles_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}

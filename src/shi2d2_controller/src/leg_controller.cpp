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
      timer_ = this->create_wall_timer(CONTROLLER_LOOP_PERIOD_MS * 1ms, std::bind(&LegController::timer_callback, this));

      start_time_ = rclcpp::Time();
      clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", CONTROLLER_LOOP_PERIOD_MS,
        [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {
            clock_callback(msg);
        });

      init_robot();
      
      RCLCPP_INFO(this->get_logger(), "FINISHED INIT...");
    }

  private:
    void solve_leg_ik(int leg_id, FootPose foot_pose, LegJointAngles &leg_joint_angles)
    {
      // shift from origin at foot to origin at body-upper-hip joint
      double x = -foot_pose.position.x + BODY_TO_FOOT_DISTANCE_MM.x;
      double y = foot_pose.position.y + BODY_TO_FOOT_DISTANCE_MM.y;
      double z = -foot_pose.position.z + BODY_TO_FOOT_DISTANCE_MM.z - UPPER_HIP_LINK_LENGTH_MM - FOOT_LINK_LENGTH_MM;
      double pitch = foot_pose.rotation.pitch;
      double yaw = foot_pose.rotation.yaw;

      // if foot has non-zero yaw orientation, rotate the goal position to align with the new orientation
      if (yaw != 0) {
        double x0 = foot_pose.position.x + BODY_TO_FOOT_DISTANCE_MM.x;
        double y0 = foot_pose.position.y + BODY_TO_FOOT_DISTANCE_MM.y;
        x = (cos(-yaw) * x0 + sin(-yaw) * y0) * -1;
        y = (-sin(-yaw) * x0 + cos(-yaw) * y0) * 1;
      }

      double leg_len_sq = x*x + y*y + z*z - (LOWER_HIP_LINK_LENGTH_MM*LOWER_HIP_LINK_LENGTH_MM * (x*x + y*y + z*z))/(y*y + z*z);
      double foot_to_hip_len_sq = y*y + z*z;
      
      double hip_roll_angle1 = acos(LOWER_HIP_LINK_LENGTH_MM/sqrt(foot_to_hip_len_sq));
      double hip_roll_angle2 = acos(z/sqrt(foot_to_hip_len_sq));
      
      double knee_bend_angle_num = leg_len_sq - UPPER_LEG_LINK_LENGTH_MM*UPPER_LEG_LINK_LENGTH_MM - LOWER_LEG_LINK_LENGTH_MM*LOWER_LEG_LINK_LENGTH_MM;
      double knee_bend_angle_den = -2 * UPPER_LEG_LINK_LENGTH_MM * LOWER_LEG_LINK_LENGTH_MM;
      double knee_bend_angle = acos(knee_bend_angle_num/knee_bend_angle_den);

      leg_joint_angles.upper_hip_body_joint_angle = yaw;
      leg_joint_angles.lower_hip_upper_hip_joint_angle = hip_roll_angle1 + hip_roll_angle2 - M_PI/2.0;
      leg_joint_angles.lower_leg_upper_leg_joint_angle = -(M_PI - knee_bend_angle);
      leg_joint_angles.upper_leg_lower_hip_joint_angle = atan2(x, z) + asin(LOWER_LEG_LINK_LENGTH_MM * sin(knee_bend_angle)/sqrt(leg_len_sq));
      leg_joint_angles.foot_lower_leg_joint_angle = -leg_joint_angles.upper_leg_lower_hip_joint_angle - leg_joint_angles.lower_leg_upper_leg_joint_angle + pitch;

      // std::cout << " HIP1: " << (leg_joint_angles.upper_hip_body_joint_angle * 180.0/M_PI)
      //           << " HIP2: " << (leg_joint_angles.lower_hip_upper_hip_joint_angle * 180.0/M_PI)
      //           << " HIP3: " << (leg_joint_angles.upper_leg_lower_hip_joint_angle * 180.0/M_PI)
      //           << " KNEE: " << (leg_joint_angles.lower_leg_upper_leg_joint_angle * 180.0/M_PI) 
      //           << " ANKLE: " << (leg_joint_angles.foot_lower_leg_joint_angle * 180.0/M_PI)
      //           << "\n"
      //           << std::endl;
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
        left_leg_joint_traj_point_.positions[joint_id] = angle * LEFT_LEG_JOINT_SIGNS[joint_id] + LEFT_LEG_JOINT_OFFSETS[joint_id];
      } else if (leg_id == RIGHT_LEG) {
        right_leg_joint_traj_point_.positions[joint_id] = angle * RIGHT_LEG_JOINT_SIGNS[joint_id] + RIGHT_LEG_JOINT_OFFSETS[joint_id];
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

    void init_robot()
    {
      for (int i = 0; i < NUM_LEG_JOINTS; i++) {
        std::cout << i << ": " << LEFT_LEG_JOINTS[i] << " and " << RIGHT_LEG_JOINTS[i] << std::endl;
        left_leg_joint_traj_.joint_names.push_back(LEFT_LEG_JOINTS[i]);
        left_leg_joint_traj_point_.positions.push_back(0.0);
        right_leg_joint_traj_.joint_names.push_back(RIGHT_LEG_JOINTS[i]);
        right_leg_joint_traj_point_.positions.push_back(0.0);
      }

      LegJointAngles left_leg_joint_angles;
      LegJointAngles right_leg_joint_angles;

      solve_leg_ik(LEFT_LEG, DEFAULT_FOOT_POSE, left_leg_joint_angles);
      solve_leg_ik(RIGHT_LEG, DEFAULT_FOOT_POSE, right_leg_joint_angles);
      set_leg_joint_angles(LEFT_LEG, left_leg_joint_angles);
      set_leg_joint_angles(RIGHT_LEG, right_leg_joint_angles);
      
      write_leg_angles();
    }

    void timer_callback()
    {
      if (sim_time_elapsed_sec_ < 2.0) {
        return;
      }
      
      double yaw = ((int) count_/200) % 2 == 0 ? 0.0 : M_PI/6.0 * 0.0;

      // double fx = ((int) count_/200) % 2 == 0 ? 40.0 : (40.0 * cos(yaw) + LOWER_HIP_LINK_LENGTH_MM * sin(yaw));
      // double fy = ((int) count_/200) % 2 == 0 ? 0.0 : (40.0 * -sin(yaw) + LOWER_HIP_LINK_LENGTH_MM * cos(yaw)) - LOWER_HIP_LINK_LENGTH_MM;
      // double fz = ((int) count_/200) % 2 == 0 ? 80.0 : 80.0;

      // double fx = ((int) count_/200) % 2 == 0 ? 0.0 : 0.0;
      // double fy = ((int) count_/200) % 2 == 0 ? 0.0 : 0.0;
      // double fz = ((int) count_/200) % 2 == 0 ? 0.0 : 80.0;

      // double yaw = 0;
      // double fx = 40.0 * cos(sim_time_elapsed_sec_ * 1.5 * M_PI);
      // double fy = 40.0 * sin(sim_time_elapsed_sec_ * 1.5 * M_PI) + 40.0*0.0;
      // double fz = 20.0 * sin(sim_time_elapsed_sec_ * 1.5 * M_PI) + 40.0 + 20.0;

      // Position foot_position = {fx, fy, fz};
      // Rotation foot_rotation = {0.0, 0.0, yaw};
      // FootPose foot_pose = {foot_position, foot_rotation};

      double left_foot_x = DEFAULT_FOOT_POSITION.x;
      double left_foot_y = DEFAULT_FOOT_POSITION.y;
      double left_foot_z = DEFAULT_FOOT_POSITION.z;
      double left_foot_yaw = DEFAULT_FOOT_ROTATION.yaw;

      double right_foot_x = DEFAULT_FOOT_POSITION.x;
      double right_foot_y = DEFAULT_FOOT_POSITION.y;
      double right_foot_z = DEFAULT_FOOT_POSITION.z;
      double right_foot_yaw = DEFAULT_FOOT_ROTATION.yaw;

      int walk_start_tick = 0; // change to be nonzero at some point
      int step_tick = count_ - walk_start_tick;

      int count_time_elapsed_ms = (step_tick * CONTROLLER_LOOP_PERIOD_MS);
      int step_half_cycle_count = (count_time_elapsed_ms/((int) STEP_PERIOD_MS/2)) % 2;
      int step_half_cycle_time_ms = count_time_elapsed_ms % ((int) STEP_PERIOD_MS/2);

      double step_slide_x = DEFAULT_FOOT_POSITION.x + STEP_LENGTH_MM/2 - STEP_SLIDE_SPEED_MM_P_S * step_half_cycle_time_ms;
      double step_slide_y = DEFAULT_FOOT_POSITION.y;
      double step_slide_z = DEFAULT_FOOT_POSITION.z;
      double step_slide_yaw = DEFAULT_FOOT_ROTATION.yaw;

      double step_lift_x = DEFAULT_FOOT_POSITION.x - STEP_LENGTH_MM * cos(M_PI/(STEP_PERIOD_MS * 0.5) * step_half_cycle_time_ms);
      double step_lift_y = DEFAULT_FOOT_POSITION.y;
      double step_lift_z = DEFAULT_FOOT_POSITION.z + STEP_LENGTH_MM * sin(M_PI/(STEP_PERIOD_MS * 0.5) * step_half_cycle_time_ms);
      double step_lift_yaw = DEFAULT_FOOT_ROTATION.yaw;
      
      std::cout << count_time_elapsed_ms << ": " << step_half_cycle_time_ms << ", " << step_half_cycle_count << std::endl;
      // std::cout << "SLIDE: " << step_slide_x << ", " << step_slide_z << " & " << step_lift_x << ", " << step_lift_z << std::endl;
      

      Position left_foot_position;
      Rotation left_foot_rotation;
      Position right_foot_position;
      Rotation right_foot_rotation;

      if (step_half_cycle_count == 0) {
        left_foot_position = {step_slide_x, step_slide_y, step_slide_z};
        left_foot_rotation = {0.0, 0.0, step_slide_yaw};
        right_foot_position = {step_lift_x, step_lift_y, step_lift_z};
        right_foot_rotation = {0.0, 0.0, step_lift_yaw};
      } else if (step_half_cycle_count == 1) {
        left_foot_position = {step_lift_x, step_lift_y, step_lift_z};
        left_foot_rotation = {0.0, 0.0, step_lift_yaw};
        right_foot_position = {step_slide_x, step_slide_y, step_slide_z};
        right_foot_rotation = {0.0, 0.0, step_slide_yaw};
      } else {
        left_foot_position = DEFAULT_FOOT_POSITION;
        left_foot_rotation = DEFAULT_FOOT_ROTATION;
        right_foot_position = DEFAULT_FOOT_POSITION;
        right_foot_rotation = DEFAULT_FOOT_ROTATION;  
      }

      FootPose left_foot_pose = {left_foot_position, left_foot_rotation};
      FootPose right_foot_pose = {right_foot_position, right_foot_rotation};
      LegJointAngles left_leg_joint_angles;
      LegJointAngles right_leg_joint_angles;

      // left_leg_joint_angles.upper_hip_body_joint_angle = 25.0 * M_PI/180.0;
      // left_leg_joint_angles.lower_hip_upper_hip_joint_angle = 25.0 * M_PI/180.0;
      // left_leg_joint_angles.upper_leg_lower_hip_joint_angle = 25.0 * M_PI/180.0;
      // left_leg_joint_angles.lower_leg_upper_leg_joint_angle = 25.0 * M_PI/180.0;
      // left_leg_joint_angles.foot_lower_leg_joint_angle = 25.0 * M_PI/180.0;

      // right_leg_joint_angles.upper_hip_body_joint_angle = 25.0 * M_PI/180.0;
      // right_leg_joint_angles.lower_hip_upper_hip_joint_angle = 25.0 * M_PI/180.0;
      // right_leg_joint_angles.upper_leg_lower_hip_joint_angle = 25.0 * M_PI/180.0;
      // right_leg_joint_angles.lower_leg_upper_leg_joint_angle = 25.0 * M_PI/180.0;
      // right_leg_joint_angles.foot_lower_leg_joint_angle = 25.0 * M_PI/180.0;

      solve_leg_ik(LEFT_LEG, left_foot_pose, left_leg_joint_angles);
      solve_leg_ik(RIGHT_LEG, right_foot_pose, right_leg_joint_angles);
      set_leg_joint_angles(LEFT_LEG, left_leg_joint_angles);
      set_leg_joint_angles(RIGHT_LEG, right_leg_joint_angles);
      
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

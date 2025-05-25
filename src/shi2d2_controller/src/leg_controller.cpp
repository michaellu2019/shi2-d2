#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "shi2d2_controller/shi2d2_controller.hpp"

#include "shi2d2_interfaces/msg/foot_pose.hpp"
#include "shi2d2_interfaces/msg/leg_joint_angles.hpp"

using namespace std::chrono_literals;

class LegController : public rclcpp::Node
{
  public:
    LegController() : Node("leg_controller"), tick_count_(0)
    {
      left_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("left_leg_group_controller/joint_trajectory", 10);
      right_leg_joint_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("right_leg_group_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(CONTROLLER_LOOP_PERIOD_MS * 1ms, std::bind(&LegController::timer_callback, this));

      start_time_ = rclcpp::Time();
      clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", CONTROLLER_LOOP_PERIOD_MS,
        std::bind(&LegController::clock_callback, this, std::placeholders::_1));

      teleop_subscriber_ = this->create_subscription<std_msgs::msg::Int8>("teleop_commands", CONTROLLER_LOOP_PERIOD_MS,
        std::bind(&LegController::teleop_callback, this, std::placeholders::_1));


      init_robot();
      
      RCLCPP_INFO(this->get_logger(), "FINISHED INIT...");
    }

    ~LegController()
    {
      return;
    }

  private:
  void init_robot()
    {
      for (int i = 0; i < NUM_LEG_JOINTS; i++) {
        std::cout << i << ": " << LEFT_LEG_JOINTS[i] << " and " << RIGHT_LEG_JOINTS[i] << std::endl;
        left_leg_joint_traj_.joint_names.push_back(LEFT_LEG_JOINTS[i]);
        left_leg_joint_traj_point_.positions.push_back(0.0);
        right_leg_joint_traj_.joint_names.push_back(RIGHT_LEG_JOINTS[i]);
        right_leg_joint_traj_point_.positions.push_back(0.0);
      }
      set_foot_pose_values(DEFAULT_FOOT_POSE_, 
                           DEFAULT_FOOT_POSITION_X_M, DEFAULT_FOOT_POSITION_Y_M, DEFAULT_FOOT_POSITION_Z_M,
                           DEFAULT_FOOT_ROTATION_RX_RAD, DEFAULT_FOOT_ROTATION_RY_RAD, DEFAULT_FOOT_ROTATION_RZ_RAD);

      shi2d2_interfaces::msg::LegJointAngles left_leg_joint_angles;
      shi2d2_interfaces::msg::LegJointAngles right_leg_joint_angles;

      solve_leg_ik(LEFT_LEG, DEFAULT_FOOT_POSE_, left_leg_joint_angles);
      solve_leg_ik(RIGHT_LEG, DEFAULT_FOOT_POSE_, right_leg_joint_angles);
      set_leg_joint_angles(LEFT_LEG, left_leg_joint_angles);
      set_leg_joint_angles(RIGHT_LEG, right_leg_joint_angles);
      
      write_leg_angles();
    }

    void set_foot_pose_values(shi2d2_interfaces::msg::FootPose &foot_pose1, const shi2d2_interfaces::msg::FootPose &foot_pose2)
    {
      foot_pose1.x = foot_pose2.x;
      foot_pose1.y = foot_pose2.y;
      foot_pose1.z = foot_pose2.z;
      foot_pose1.rx = foot_pose2.rx;
      foot_pose1.ry = foot_pose2.ry;
      foot_pose1.rz = foot_pose2.rz;
    }

    void set_foot_pose_values(shi2d2_interfaces::msg::FootPose &foot_pose, double x, double y, double z, double rx, double ry, double rz)
    {
      foot_pose.x = x;
      foot_pose.y = y;
      foot_pose.z = z;
      foot_pose.rx = rx;
      foot_pose.ry = ry;
      foot_pose.rz = rz;
    }

    void solve_leg_ik(int leg_id, shi2d2_interfaces::msg::FootPose foot_pose, shi2d2_interfaces::msg::LegJointAngles &leg_joint_angles)
    {
      // shift from origin at foot to origin at body-upper-hip joint
      double x = -foot_pose.x + BODY_TO_FOOT_DISTANCE_X_M;
      double y = foot_pose.y + BODY_TO_FOOT_DISTANCE_Y_M;
      double z = -foot_pose.z + BODY_TO_FOOT_DISTANCE_Z_M - UPPER_HIP_LINK_LENGTH_M;

      double roll = 0.0; //foot_pose.rx; // TODO consider how foot roll influences x, y, z of foot placement
      double pitch = 0.0; //foot_pose.ry; // TODO consider how foot pitch influences x, y, z of foot placement
      double yaw = foot_pose.rz;

      // if foot has non-zero yaw orientation, rotate the goal foot position to align with the new orientation
      // this simplifies the math for the rest of the IK solver
      if (yaw != 0) {
        double x0 = foot_pose.x + BODY_TO_FOOT_DISTANCE_X_M;
        double y0 = foot_pose.y + BODY_TO_FOOT_DISTANCE_Y_M;
        x = (x0 * cos(-yaw) + y0 * sin(-yaw)) * -1;
        y = (-x0 * sin(-yaw) + y0 * cos(-yaw)) * 1;
      }

      // solve for hip roll angle
      // "projected" indicates a distance projected to the "frontal", "coronal", or YZ plane of the robot
      double projected_foot_to_roll_hip_dist = sqrt(y*y + z*z);
      
      double hip_roll_angle1 = acos(LOWER_HIP_LINK_LENGTH_M/projected_foot_to_roll_hip_dist);
      double hip_roll_angle2 = acos(z/projected_foot_to_roll_hip_dist);
      double hip_roll_angle = hip_roll_angle1 + hip_roll_angle2 - M_PI/2.0;

      // solve for the position of the ankle joint given the new hip roll angle
      double x_ankle = x;
      double y_ankle = y - FOOT_LINK_LENGTH_M * sin(hip_roll_angle);
      double z_ankle = z - FOOT_LINK_LENGTH_M * cos(hip_roll_angle);

      // solve for hip pitch, knee, and ankle angles
      // find the relevant distances from the upper-leg-lower-hip joint to the ankle joint and foot
      double projected_foot_to_hip_dist = projected_foot_to_roll_hip_dist * sin(hip_roll_angle1);
      double projected_ankle_to_hip_dist = projected_foot_to_hip_dist - FOOT_LINK_LENGTH_M;
      double ankle_to_hip_dist = sqrt(x_ankle*x_ankle + projected_ankle_to_hip_dist*projected_ankle_to_hip_dist);
      
      // use the law of cosines to solve for the knee bend angle
      double knee_bend_angle_num = ankle_to_hip_dist*ankle_to_hip_dist - UPPER_LEG_LINK_LENGTH_M*UPPER_LEG_LINK_LENGTH_M - LOWER_LEG_LINK_LENGTH_M*LOWER_LEG_LINK_LENGTH_M;
      double knee_bend_angle_den = -2 * UPPER_LEG_LINK_LENGTH_M * LOWER_LEG_LINK_LENGTH_M;
      double knee_bend_angle = acos(knee_bend_angle_num/knee_bend_angle_den);

      // use more trigonometry and the law of sines to solve for the hip pitch bend angle
      double hip_bend_angle1 = atan2(x_ankle, projected_ankle_to_hip_dist);
      double hip_bend_angle2 = asin(LOWER_LEG_LINK_LENGTH_M/ankle_to_hip_dist * sin(knee_bend_angle));

      leg_joint_angles.upper_hip_body_joint_angle = yaw;
      leg_joint_angles.lower_hip_upper_hip_joint_angle = hip_roll_angle;
      leg_joint_angles.upper_leg_lower_hip_joint_angle = hip_bend_angle1 + hip_bend_angle2;
      leg_joint_angles.lower_leg_upper_leg_joint_angle = -(M_PI - knee_bend_angle);
      leg_joint_angles.foot_lower_leg_joint_angle = -leg_joint_angles.upper_leg_lower_hip_joint_angle - leg_joint_angles.lower_leg_upper_leg_joint_angle + pitch;

      // std::cout << "HIP1: " << (leg_joint_angles.upper_hip_body_joint_angle * 180.0/M_PI)
      //           << ", HIP2: " << (leg_joint_angles.lower_hip_upper_hip_joint_angle * 180.0/M_PI)
      //           << ", HIP3: " << (leg_joint_angles.upper_leg_lower_hip_joint_angle * 180.0/M_PI)
      //           << ", KNEE: " << (leg_joint_angles.lower_leg_upper_leg_joint_angle * 180.0/M_PI) 
      //           << ", ANKLE: " << (leg_joint_angles.foot_lower_leg_joint_angle * 180.0/M_PI)
      //           << std::endl;
    }

    void test_leg_ik(int leg_id, shi2d2_interfaces::msg::FootPose &foot_pose)
    {
      int swap_frequency = 200;
    
      // hip flexors ik test
      // double yaw = ((int) tick_count_/swap_frequency) % 2 == 0 ? 0.0 : M_PI/6.0;
      // double fx = ((int) tick_count_/swap_frequency) % 2 == 0 ? 40.0 : (40.0 * cos(yaw) + LOWER_HIP_LINK_LENGTH_M * sin(yaw));
      // double fy = ((int) tick_count_/swap_frequency) % 2 == 0 ? 0.0 : (40.0 * -sin(yaw) + LOWER_HIP_LINK_LENGTH_M * cos(yaw)) - LOWER_HIP_LINK_LENGTH_M;
      // double fz = ((int) tick_count_/swap_frequency) % 2 == 0 ? 60.0 : 60.0;

      // squats ik test
      double yaw = 0;
      double fx = ((int) tick_count_/swap_frequency) % 2 == 0 ? 0.0 : 0.0;
      double fy = ((int) tick_count_/swap_frequency) % 2 == 0 ? 0.0 : 0.0;
      // double fz = ((int) tick_count_/swap_frequency) % 2 == 0 ? 0.0 : 60.0;
      double fz = 30.0 * sin(sim_time_elapsed_sec_ * (swap_frequency/160) * M_PI) + 30.0;

      // leg extension ik test
      // double yaw = 0;
      // double fx = 40.0 * cos(sim_time_elapsed_sec_ * (swap_frequency/160) * M_PI) + 0.0;
      // double fy = 0.0 * sin(sim_time_elapsed_sec_ * (swap_frequency/160) * M_PI) + 0.0;
      // double fz = 0.0 * sin(sim_time_elapsed_sec_ * (swap_frequency/160) * M_PI) + 60.0;
      // std::cout << "TICK: " << tick_count_ << ", fx: " << fx << ", fy: " << fy << ", fz: " << fz << std::endl;

      set_foot_pose_values(foot_pose, fx, fy, fz, 0.0, 0.0, yaw);
    }

    void set_leg_joint_angles(int leg_id, shi2d2_interfaces::msg::LegJointAngles &leg_joint_angles)
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

    void walk_open_loop(int walking_direction, shi2d2_interfaces::msg::FootPose &left_foot_pose, shi2d2_interfaces::msg::FootPose &right_foot_pose)
    {
      int step_tick = tick_count_ - walk_start_tick_;
      double step_angle_rad = TURN_STEP_ANGLE_RAD * ((walking_direction == CLOCKWISE) - (walking_direction == COUNTERCLOCKWISE));
      double step_length_m = FORWARD_STEP_LENGTH_M * ((walking_direction == FORWARD) - (walking_direction == BACKWARD));
      double step_width_m = SIDE_STEP_WIDTH_M * ((walking_direction == LEFT) - (walking_direction == RIGHT));
      double step_height_m = STEP_HEIGHT_M * 1.0;
      double step_x_slide_speed_m_p_s = step_length_m/(STEP_PERIOD_MS * 0.5);
      double step_y_slide_speed_m_p_s = step_width_m/(STEP_PERIOD_MS * 0.5);
      double step_turn_speed_rad_p_s = step_angle_rad/(STEP_PERIOD_MS * 0.5);

      int count_time_elapsed_ms = (step_tick * CONTROLLER_LOOP_PERIOD_MS);
      int step_half_cycle_count = (count_time_elapsed_ms/((int) STEP_PERIOD_MS/2)) % 2;
      int step_half_cycle_time_ms = count_time_elapsed_ms % ((int) STEP_PERIOD_MS/2);

      double yaw = -step_angle_rad/2 + step_turn_speed_rad_p_s * step_half_cycle_time_ms;
      
      double step_slide_x = DEFAULT_FOOT_POSITION_X_M + step_length_m/2 - step_x_slide_speed_m_p_s * step_half_cycle_time_ms;
      double step_slide_y = DEFAULT_FOOT_POSITION_Y_M + step_width_m/2 - step_y_slide_speed_m_p_s * step_half_cycle_time_ms;
      double step_slide_z = DEFAULT_FOOT_POSITION_Z_M;
      if (yaw != 0) {
        step_slide_x = DEFAULT_FOOT_POSITION_X_M * cos(-yaw) + (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M) * sin(-yaw);
        step_slide_y = (DEFAULT_FOOT_POSITION_X_M * -sin(-yaw) + (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M) * cos(-yaw)) - (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M);
      }
      double step_slide_yaw = yaw;

      double step_lift_x = DEFAULT_FOOT_POSITION_X_M - step_length_m * cos(M_PI/(STEP_PERIOD_MS * 0.5) * step_half_cycle_time_ms);
      double step_lift_y = DEFAULT_FOOT_POSITION_Y_M - step_width_m * cos(M_PI/(STEP_PERIOD_MS * 0.5) * step_half_cycle_time_ms);
      double step_lift_z = DEFAULT_FOOT_POSITION_Z_M + step_height_m * sin(M_PI/(STEP_PERIOD_MS * 0.5) * step_half_cycle_time_ms);
      if (yaw != 0) {
        step_lift_x = DEFAULT_FOOT_POSITION_X_M * cos(yaw) + (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M) * sin(yaw);
        step_lift_y = (DEFAULT_FOOT_POSITION_X_M * -sin(yaw) + (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M) * cos(yaw)) - (DEFAULT_FOOT_POSITION_Y_M + LOWER_HIP_LINK_LENGTH_M);
      }
      double step_lift_yaw = yaw;
      
      // std::cout << count_time_elapsed_ms << ": " << step_half_cycle_time_ms << ", " << step_half_cycle_count << std::endl;
      // std::cout << "SLIDE: " << step_slide_x << ", " << step_slide_z << " & " << step_lift_x << ", " << step_lift_z << std::endl;
      
      if (step_half_cycle_count == 0) {
        set_foot_pose_values(left_foot_pose, step_slide_x, step_slide_y, step_slide_z, 0.0, 0.0, -step_slide_yaw);
        set_foot_pose_values(right_foot_pose, step_lift_x, -step_lift_y, step_lift_z, 0.0, 0.0, step_lift_yaw);
      } else if (step_half_cycle_count == 1) {
        set_foot_pose_values(left_foot_pose, step_lift_x, step_lift_y, step_lift_z, 0.0, 0.0, -step_lift_yaw);
        set_foot_pose_values(right_foot_pose, step_slide_x, -step_slide_y, step_slide_z, 0.0, 0.0, step_slide_yaw);
      } else {
        set_foot_pose_values(left_foot_pose, DEFAULT_FOOT_POSE_);
        set_foot_pose_values(right_foot_pose, DEFAULT_FOOT_POSE_);
      }
    }

    void timer_callback()
    {
      if (sim_time_elapsed_sec_ < 1.0) {
        return;
      }

      shi2d2_interfaces::msg::FootPose left_foot_pose = DEFAULT_FOOT_POSE_;
      shi2d2_interfaces::msg::FootPose right_foot_pose = DEFAULT_FOOT_POSE_;
      shi2d2_interfaces::msg::LegJointAngles left_leg_joint_angles;
      shi2d2_interfaces::msg::LegJointAngles right_leg_joint_angles;

      if ((tick_count_ - last_teleop_command_tick_) * CONTROLLER_LOOP_PERIOD_MS > TELEOP_COMMAND_TIMEOUT_MS) {
        teleop_command_ = STOP;
      }
      if (teleop_command_ != STOP) {
        walk_open_loop(teleop_command_, left_foot_pose, right_foot_pose);
      }

      // test_leg_ik(LEFT_LEG, left_foot_pose);
      // test_leg_ik(RIGHT_LEG, right_foot_pose);

      solve_leg_ik(LEFT_LEG, left_foot_pose, left_leg_joint_angles);
      solve_leg_ik(RIGHT_LEG, right_foot_pose, right_leg_joint_angles);
      set_leg_joint_angles(LEFT_LEG, left_leg_joint_angles);
      set_leg_joint_angles(RIGHT_LEG, right_leg_joint_angles);
      
      write_leg_angles();

      tick_count_++;
    }

    void teleop_callback(std_msgs::msg::Int8::SharedPtr msg)
    {
      int walking_direction = msg->data;
      if (walking_direction != teleop_command_) {
        walk_start_tick_ = tick_count_;
      }
      last_teleop_command_tick_ = tick_count_;
      teleop_command_ = walking_direction;
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
      if (start_time_.nanoseconds() == 0) {
          start_time_ = msg->clock;
      }
      sim_time_ = msg->clock;
      
      if (start_time_.nanoseconds() != 0) {
        rclcpp::Duration sim_time_elapsed_ = sim_time_ - start_time_;
        sim_time_elapsed_sec_ = sim_time_elapsed_.seconds();
        // RCLCPP_INFO(this->get_logger(), "Time since simulation start: %.2fs...", sim_time_elapsed_sec_);
      }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_leg_joint_traj_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_leg_joint_traj_publisher_;

    trajectory_msgs::msg::JointTrajectory left_leg_joint_traj_ = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint left_leg_joint_traj_point_ = trajectory_msgs::msg::JointTrajectoryPoint();
    trajectory_msgs::msg::JointTrajectory right_leg_joint_traj_ = trajectory_msgs::msg::JointTrajectory();
    trajectory_msgs::msg::JointTrajectoryPoint right_leg_joint_traj_point_ = trajectory_msgs::msg::JointTrajectoryPoint();

    shi2d2_interfaces::msg::FootPose DEFAULT_FOOT_POSE_;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t tick_count_;
    size_t walk_start_tick_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Time start_time_;
    rclcpp::Time sim_time_;
    double sim_time_elapsed_sec_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr teleop_subscriber_;
    int teleop_command_;
    size_t last_teleop_command_tick_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegController>());
  rclcpp::shutdown();
  return 0;
}

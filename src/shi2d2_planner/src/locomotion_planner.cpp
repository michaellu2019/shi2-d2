#include <chrono>
#include <cmath>
#include <nlopt.hpp>
#include <matplotlibcpp.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "shi2d2_interfaces/msg/foot_pose.hpp"

#include "shi2d2_planner/shi2d2_planner.hpp"

using namespace std::chrono_literals;
namespace plt = matplotlibcpp;

class LocomotionPlanner : public rclcpp::Node
{
public:
  LocomotionPlanner() : Node("locomotion_planner")
  {
    // Publisher
    publisher_ = this->create_publisher<std_msgs::msg::Int8>("locomotion_commands", PLANNER_LOOP_PERIOD_MS);
    foot_pose_publisher_ = this->create_publisher<shi2d2_interfaces::msg::FootPose>("foot_pose", FOOT_POSE_PUBLISHER_LOOP_PERIOD_MS);

    // Subscribers
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/shi2d2/imu", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::imu_callback, this, std::placeholders::_1));

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/shi2d2/odometry", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::odometry_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(PLANNER_LOOP_PERIOD_MS * 1ms, std::bind(&LocomotionPlanner::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    init_zmp_mpc();
    solve_zmp_mpc();
  }

  ~LocomotionPlanner()
  {
    return;
  }

private:
  void quaternion_to_euler(double &x, double &y, double &z, double &w, double &roll, double &pitch, double &yaw) {
    tf2::Quaternion tf_q(x, y, z, w);
    tf2::Matrix3x3 m(tf_q);
    m.getRPY(roll, pitch, yaw);
  }

  void get_robot_com()
  {
    
  }

  void get_robot_zmp()
  {

  }

  void get_robot_state()
  {
    // Get the robot state from the IMU and odometry data
    body_state_.x = body_odometry_.pose.pose.position.x;
    body_state_.y = body_odometry_.pose.pose.position.y;
    body_state_.z = body_odometry_.pose.pose.position.z;

    quaternion_to_euler(body_imu_data_.orientation.x, body_imu_data_.orientation.y, body_imu_data_.orientation.z, body_imu_data_.orientation.w,
                        body_state_.rx, body_state_.ry, body_state_.rz);

    body_state_.vx = body_odometry_.twist.twist.linear.x;
    body_state_.vy = body_odometry_.twist.twist.linear.y;
    body_state_.vz = body_odometry_.twist.twist.linear.z;

    body_state_.wx = body_imu_data_.angular_velocity.x;
    body_state_.wy = body_imu_data_.angular_velocity.y;
    body_state_.wz = body_imu_data_.angular_velocity.z;

    body_state_.ax = body_imu_data_.linear_acceleration.x;
    body_state_.ay = body_imu_data_.linear_acceleration.y;
    body_state_.az = body_imu_data_.linear_acceleration.z;

    body_state_.alx = 0.0; // TODO: is there a way to get angular acceleration if we need it?
    body_state_.aly = 0.0;
    body_state_.alz = 0.0;
    
    // std::cout << std::fixed << std::setprecision(3)
    //       << "Body State: X: [" 
    //       << body_state_.x << ", " << body_state_.y << ", " << body_state_.z << ", "
    //       << body_state_.rx << ", " << body_state_.ry << ", " << body_state_.rz << "] XD: ["
    //       << body_state_.vx << ", " << body_state_.vy << ", " << body_state_.vz << ", "
    //       << body_state_.wx << ", " << body_state_.wy << ", " << body_state_.wz << "] XDD:["
    //       << body_state_.ax << ", " << body_state_.ay << ", " << body_state_.az << ", "
    //       << body_state_.alx << ", " << body_state_.aly << ", " << body_state_.alz << "]" << std::endl;
  }

  void plan_footsteps()
  {
    t_.clear();
    left_footstep_positions_.clear();
    right_footstep_positions_.clear();
    zmp_refs_x_.clear();
    zmp_refs_y_.clear();

    // generate footsteps and zmp reference positions
    for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      int t_walk_ms = std::max(0, (int) (t_ms - ZMP_MPC_NUM_STATIONARY_TIMESTEPS * ZMP_MPC_TIMESTEP_DURATION_MS));
      t_.push_back(t_ms);

      geometry_msgs::msg::Vector3 left_foot_position;
      geometry_msgs::msg::Vector3 right_foot_position;

      if (i < ZMP_MPC_NUM_STATIONARY_TIMESTEPS) {
        left_foot_position.x = body_state_.x;
        left_foot_position.y = body_state_.y + ZMP_STEP_WIDTH_M;
        left_foot_position.z = 0.0;

        right_foot_position.x = body_state_.x;
        right_foot_position.y = body_state_.y - ZMP_STEP_WIDTH_M;
        right_foot_position.z = 0.0;

        left_footstep_positions_.push_back(left_foot_position);
        right_footstep_positions_.push_back(right_foot_position);

        double zmp_ref_x = (left_foot_position.x + right_foot_position.x)/2;
        double zmp_ref_y = (left_foot_position.y + right_foot_position.y)/2;
        zmp_refs_x_.push_back(zmp_ref_x);
        zmp_refs_y_.push_back(zmp_ref_y);
      } else if (i > ZMP_MPC_NUM_TIMESTEPS - ZMP_MPC_NUM_STATIONARY_TIMESTEPS) {
        double last_step_x = body_state_.x + ((ZMP_MPC_NUM_TIMESTEPS - ZMP_MPC_NUM_STATIONARY_TIMESTEPS) * ZMP_MPC_TIMESTEP_DURATION_MS)/step_period_ms * ZMP_STEP_LENGTH_M;

        left_foot_position.x = last_step_x;
        left_foot_position.y = body_state_.y + ZMP_STEP_WIDTH_M;
        left_foot_position.z = 0.0;
        
        right_foot_position.x = last_step_x;
        right_foot_position.y = body_state_.y - ZMP_STEP_WIDTH_M;
        right_foot_position.z = 0.0;

        left_footstep_positions_.push_back(left_foot_position);
        right_footstep_positions_.push_back(right_foot_position);


        double zmp_ref_x = (left_foot_position.x + right_foot_position.x)/2;
        double zmp_ref_y = (left_foot_position.y + right_foot_position.y)/2;
        zmp_refs_x_.push_back(zmp_ref_x);
        zmp_refs_y_.push_back(zmp_ref_y);
      } else {
        int left_step_num = (int) (t_walk_ms + half_step_period_ms)/step_period_ms;
        int right_step_num = (int) t_walk_ms/step_period_ms;
        bool left_foot_floating = false;
        bool right_foot_floating = false;
  

        // left footstep generation
        double left_foot_x_offset = body_state_.x;
        double left_foot_x_slope = ZMP_STEP_LENGTH_M;
        if ((t_walk_ms + (int) half_step_period_ms) % (int) step_period_ms < step_support_duration_ms) {
          left_foot_position.x = left_foot_x_offset + left_step_num * ZMP_STEP_LENGTH_M;
          left_foot_position.y = body_state_.y + ZMP_STEP_WIDTH_M;
          left_foot_position.z = 0.0;
        } else {
          left_foot_position.x = left_foot_x_offset + left_step_num * ZMP_STEP_LENGTH_M + (left_foot_x_slope * ((t_walk_ms + (int) half_step_period_ms) % (int) step_period_ms - step_support_duration_ms))/single_support_duration_ms;
          left_foot_position.y = body_state_.y + ZMP_STEP_WIDTH_M;
          left_foot_position.z = ZMP_STEP_HEIGHT_M * sin(M_PI/single_support_duration_ms * (t_walk_ms % (int) half_step_period_ms - double_support_duration_ms));
          left_foot_floating = true;
        }

        // right footstep generation
        double right_foot_x_offset = t_walk_ms < step_period_ms ? body_state_.x : body_state_.x + ZMP_STEP_LENGTH_M/2;
        double right_foot_x_slope = t_walk_ms < step_period_ms ? 1.5 * ZMP_STEP_LENGTH_M : ZMP_STEP_LENGTH_M;
        if (t_walk_ms % (int) step_period_ms < step_support_duration_ms) {
          right_foot_position.x = right_foot_x_offset + right_step_num * ZMP_STEP_LENGTH_M;
          right_foot_position.y = body_state_.y - ZMP_STEP_WIDTH_M;
          right_foot_position.z = 0.0;
        } else {
          right_foot_position.x = right_foot_x_offset + right_step_num * ZMP_STEP_LENGTH_M + (right_foot_x_slope * ((t_walk_ms % (int) step_period_ms) - step_support_duration_ms))/single_support_duration_ms;
          right_foot_position.y = body_state_.y - ZMP_STEP_WIDTH_M;
          right_foot_position.z = ZMP_STEP_HEIGHT_M * sin(M_PI/single_support_duration_ms * (t_walk_ms % (int) half_step_period_ms - double_support_duration_ms));
          right_foot_floating = true;
        }

        left_footstep_positions_.push_back(left_foot_position);
        right_footstep_positions_.push_back(right_foot_position);

        // zmp reference generation
        if (!left_foot_floating && right_foot_floating) {
          zmp_refs_x_.push_back(left_foot_position.x);
          zmp_refs_y_.push_back(left_foot_position.y);
        } else if (left_foot_floating && !right_foot_floating) {
          zmp_refs_x_.push_back(right_foot_position.x);
          zmp_refs_y_.push_back(right_foot_position.y);
        } else {
          double left_foot_y_offset = t_walk_ms < half_step_period_ms ? 0.0 : left_foot_position.y;
          double right_foot_y_offset = t_walk_ms < half_step_period_ms ? 0.0 : right_foot_position.y; 
          double zmp_ref_x = 0;
          double zmp_ref_y = 0;
          if (left_foot_position.x > right_foot_position.x) {
            zmp_ref_x = right_foot_position.x + (left_foot_position.x - right_foot_position.x) * (t_walk_ms % (int) half_step_period_ms)/double_support_duration_ms;
          } else {
            zmp_ref_x = left_foot_position.x + (right_foot_position.x - left_foot_position.x) * (t_walk_ms % (int) half_step_period_ms)/double_support_duration_ms;
          }

          if (t_walk_ms % (int) step_period_ms < double_support_duration_ms) {
            zmp_ref_y = left_foot_y_offset + (right_foot_position.y - left_foot_y_offset) * (t_walk_ms % (int) half_step_period_ms)/double_support_duration_ms;
          } else {
            zmp_ref_y = right_foot_y_offset + (left_foot_position.y - right_foot_y_offset) * (t_walk_ms % (int) half_step_period_ms)/double_support_duration_ms;
          }
          zmp_refs_x_.push_back(zmp_ref_x);
          zmp_refs_y_.push_back(zmp_ref_y);
        }
      }
    }
  }

  void init_zmp_mpc()
  {
    const int N = ZMP_MPC_NUM_TIMESTEPS;
    const double T = ZMP_MPC_TIMESTEP_DURATION_SEC;
    const double h = ZMP_MPC_COM_HEIGHT_M;
    const double Q = ZMP_MPC_COP_PENALTY;
    const double R = ZMP_MPC_U_PENALTY;

    std::cout << "Initializing ZMP MPC..." << std::endl;

    auto footstep_start = std::chrono::high_resolution_clock::now(); // Start timer

    plan_footsteps();
    
    auto footstep_end = std::chrono::high_resolution_clock::now(); // End timer
    auto footstep_duration = std::chrono::duration_cast<std::chrono::milliseconds>(footstep_start - footstep_end).count();
    std::cout << "Footstep generation took " << footstep_duration << " ms" << std::endl;

    auto zmp_gen_start = std::chrono::high_resolution_clock::now(); // Start timer

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ZMP_MPC_NUM_TIMESTEPS, ZMP_MPC_NUM_TIMESTEPS);
    Eigen::MatrixXd Pu(ZMP_MPC_NUM_TIMESTEPS, ZMP_MPC_NUM_TIMESTEPS);
    Px_.resize(ZMP_MPC_NUM_TIMESTEPS, 3);
    PuPuRQIPuT_.resize(ZMP_MPC_NUM_TIMESTEPS, ZMP_MPC_NUM_TIMESTEPS);

    // analytically solve the optimal jerk/control values for the ZMP MPC problem
    for (int i = 0; i < N; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      int n = i + 1;
      
      Px_(i, 0) = 1;
      Px_(i, 1) = n * T;
      Px_(i, 2) = pow(n * T, 2)/2 - h/g;
    }

    for (int i = 0; i < N; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      int n = i + 1;
      for (int j = 0; j <= i; j++) {
        int m = j + 1;
        int coeff = (1 + 3 * (n - m) + 3 * pow(n - m, 2));
        Pu(i, j) = coeff * pow(T, 3)/6 - (h/g) * T;
      }
    }

    PuPuRQIPuT_ = -((Pu.transpose() * Pu + (R/Q) * I).inverse()) * Pu.transpose();

    auto zmp_gen_end = std::chrono::high_resolution_clock::now(); // End timer
    auto zmp_gen_duration = std::chrono::duration_cast<std::chrono::milliseconds>(zmp_gen_end - zmp_gen_start).count();
    std::cout << "ZMP initialization took " << zmp_gen_duration << " ms" << std::endl;
  }

  void solve_zmp_mpc()
  {
    // get_robot_state(); ??????

    auto zmp_mpc_start = std::chrono::high_resolution_clock::now(); // Start timer
    
    Eigen::Vector3d X(body_state_.x, body_state_.vx, body_state_.ax);
    Eigen::Vector3d Y(body_state_.y, body_state_.vy, body_state_.ay);
    Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_X(zmp_refs_x_.data());
    Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_Y(zmp_refs_y_.data());

    Eigen::VectorXd PxZ = Px_ * X - ZMP_REFS_X;
    Eigen::VectorXd PyZ = Px_ * Y - ZMP_REFS_Y;

    Eigen::VectorXd Ux = PuPuRQIPuT_ * PxZ;
    Eigen::VectorXd Uy = PuPuRQIPuT_ * PyZ;

    auto zmp_mpc_end = std::chrono::high_resolution_clock::now(); // End timer
    auto zmp_mpc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(zmp_mpc_end - zmp_mpc_start).count();
    std::cout << "Solving ZMP MPC took " << zmp_mpc_duration << " ms" << std::endl;
    
    // calculate ZMP values based on COM values
    // std::cout << "ZMP Values" << std::endl;
    std::vector<double> com_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> com_y(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_y(ZMP_MPC_NUM_TIMESTEPS);
    Eigen::Vector3d Xp(0, 0, 0);
    Eigen::Vector3d Yp(0, 0, 0);
    int k = 0;
    
    for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      
      com_x[i] = Xp[0];
      com_y[i] = Yp[0];
      // std::cout << "(" << t_ms << ", " << com_x[i] << ", " << com_y[i] << "), ___ " << com_x.size() << ", " << com_y.size() << " ___ ";
      // std::cout << t_ms << ": " << Xp[0] << ", " << Xp[1] << ", " << Xp[2] << " - " << u_x[i - k] << std::endl;

      zmp_x[i] = (C * Xp).value();
      zmp_y[i] = (C * Yp).value();
      // std::cout << "(" << t_ms << ", " << zmp_x[i] << ", " << zmp_y[i] << "), ";

      Xp = A * Xp + B * Ux[i - k];
      Yp = A * Yp + B * Uy[i - k];
    }
    std::cout << std::endl;

    auto footpose_start = std::chrono::high_resolution_clock::now(); // Start timer

    std::vector<shi2d2_interfaces::msg::FootPose> left_foot_poses;
    std::vector<shi2d2_interfaces::msg::FootPose> right_foot_poses;

    for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;

      shi2d2_interfaces::msg::FootPose left_foot_pose; 
      left_foot_pose.leg_id = LEFT_LEG;
      left_foot_pose.x = (left_footstep_positions_[i].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      left_foot_pose.y = (0*left_footstep_positions_[i].y - com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      left_foot_pose.z = left_footstep_positions_[i].z + DEFAULT_FOOT_POSITION_Z_M;
      left_foot_pose.rx = 0.0;
      left_foot_pose.ry = 0.0;
      left_foot_pose.rz = 0.0;

      shi2d2_interfaces::msg::FootPose right_foot_pose;
      right_foot_pose.leg_id = RIGHT_LEG;
      right_foot_pose.x = (right_footstep_positions_[i].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      right_foot_pose.y = -(0*right_footstep_positions_[i].y - com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      right_foot_pose.z = right_footstep_positions_[i].z + DEFAULT_FOOT_POSITION_Z_M;
      right_foot_pose.rx = 0.0;
      right_foot_pose.ry = 0.0;
      right_foot_pose.rz = 0.0;

      left_foot_poses.push_back(left_foot_pose);
      right_foot_poses.push_back(right_foot_pose);
    }

    for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      foot_pose_publisher_->publish(left_foot_poses[i]);
      foot_pose_publisher_->publish(right_foot_poses[i]);
      rclcpp::sleep_for(std::chrono::milliseconds(5)); // Sleep to avoid flooding the topic
    }

    auto footpose_end = std::chrono::high_resolution_clock::now(); // End timer
    auto footpose_duration = std::chrono::duration_cast<std::chrono::milliseconds>(footpose_end - footpose_start).count();

    std::cout << "Published Foot Poses" << std::endl;
    std::cout << "Foot pose generation took " << footpose_duration << " ms" << std::endl;

    plot_zmp_mpc_results(
      t_, 
      com_x, com_y,
      zmp_refs_x_, zmp_refs_y_,
      zmp_x, zmp_y,
      left_footstep_positions_, right_footstep_positions_
    );
  }

  void plot_zmp_mpc_results(const std::vector<double>& time,  
    const std::vector<double>& com_x, const std::vector<double>& com_y,
    const std::vector<double>& zmp_refs_x, const std::vector<double>& zmp_refs_y,
    const std::vector<double>& zmp_x, const std::vector<double>& zmp_y,
    const std::vector<geometry_msgs::msg::Vector3>& left_footstep_positions = {},
    const std::vector<geometry_msgs::msg::Vector3>& right_footstep_positions = {}) {
    plt::figure();

    std::vector<double> left_foot_x, left_foot_y, left_foot_z;
    for (const auto& pos : left_footstep_positions) {
      left_foot_x.push_back(pos.x);
      left_foot_y.push_back(pos.y);
      left_foot_z.push_back(pos.z);
    }
    std::vector<double> right_foot_x, right_foot_y, right_foot_z;
    for (const auto& pos : right_footstep_positions) {
      right_foot_x.push_back(pos.x);
      right_foot_y.push_back(pos.y);
      right_foot_z.push_back(pos.z);
    }

    // Plot 1: COM y vs x, ZMP reference y vs x, and ZMP y vs x
    plt::subplot(3, 1, 1); // 3 rows, 1 column, 1st plot
    plt::plot(com_x, com_y, {{"label", "COM y vs x"}});
    plt::plot(zmp_refs_x, zmp_refs_y, {{"label", "ZMP Reference y vs x"}});
    plt::plot(zmp_x, zmp_y, {{"label", "ZMP y vs x"}});
    plt::xlabel("X (meters)");
    plt::ylabel("Y (meters)");
    plt::title("COM and ZMP Trajectories in X-Y Plane");
    plt::legend();
    plt::grid(true);

    // Plot 2: COM x vs t, ZMP reference x vs t, and ZMP x vs t
    plt::subplot(3, 1, 2); // 3 rows, 1 column, 2nd plot
    plt::plot(time, com_x, {{"label", "COM x vs t"}});
    plt::plot(time, zmp_refs_x, {{"label", "ZMP Reference x vs t"}});
    plt::plot(time, zmp_x, {{"label", "ZMP x vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("X (meters)");
    plt::title("COM and ZMP X Trajectories Over Time");
    plt::legend();
    plt::grid(true);

    // Plot 3: COM y vs t, ZMP reference y vs t, and ZMP y vs t
    plt::subplot(3, 1, 3); // 3 rows, 1 column, 3rd plot
    plt::plot(time, com_y, {{"label", "COM y vs t"}});
    plt::plot(time, zmp_refs_y, {{"label", "ZMP Reference y vs t"}});
    plt::plot(time, zmp_y, {{"label", "ZMP y vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("Y (meters)");
    plt::title("COM and ZMP Y Trajectories Over Time");
    plt::legend();
    plt::grid(true);

    // // Plot 2: states
    // plt::subplot(4, 1, 4); // 3 rows, 1 column, 2nd plot
    // plt::plot(time, double_support, {{"label", "DSP vs t"}});
    // plt::plot(time, right_foot_down, {{"label", "RFD vs t"}});
    // plt::xlabel("Time (seconds)");
    // plt::ylabel("States");
    // plt::title("Gait States Over Time");
    // plt::legend();
    // plt::grid(true);

    // Show all plots
    plt::tight_layout();
    plt::show();

    // 3D plot of foot step and ZMP trajectories
    // plt::figure();
    // plt::plot3(com_x, com_y, std::vector<double>(com_x.size(), 0.0), {{"label", "COM Trajectory"}});
    // plt::plot3(zmp_ref_x, zmp_ref_y, std::vector<double>(zmp_ref_x.size(), 0.0), {{"label", "ZMP Reference Trajectory"}});
    // plt::plot3(zmp_x, zmp_y, std::vector<double>(zmp_x.size(), 0.0), {{"label", "ZMP Trajectory"}});
    // plt::plot3(left_foot_x, left_foot_y, left_foot_z, {{"label", "Left Foot Steps"}});
    // plt::plot3(right_foot_x, right_foot_y, right_foot_z, {{"label", "Right Foot Steps"}});
    // plt::xlabel("X (meters)");
    // plt::ylabel("Y (meters)");
    // plt::zlabel("Z (meters)");
    // plt::title("3D Trajectories of COM and ZMP");
    // plt::legend();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    body_imu_data_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "IMU Acceleration: [%.2f, %.2f, %.2f, %.2f]",
    //             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    body_odometry_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "Odometry Position: [%.2f, %.2f, %.2f]",
    //             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void timer_callback()
  {
    // Process TF data
    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("world", "com_link", tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped left_foot_transform_stamped = tf_buffer_->lookupTransform("world", "left_foot_link", tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped right_foot_transform_stamped = tf_buffer_->lookupTransform("world", "right_foot_link", tf2::TimePointZero);
      body_transform_ = transform_stamped.transform;
      // RCLCPP_INFO(this->get_logger(), "Positions: B: [%.2f, %.2f, %.2f] L: [%.2f, %.2f, %.2f] R: [%.2f, %.2f, %.2f]",
                  // body_transform_.translation.x,
                  // body_transform_.translation.y,
                  // body_transform_.translation.z,
                  // left_foot_transform_stamped.transform.translation.x,
                  // left_foot_transform_stamped.transform.translation.y,
                  // left_foot_transform_stamped.transform.translation.z,
                  // right_foot_transform_stamped.transform.translation.x,
                  // right_foot_transform_stamped.transform.translation.y,
                  // right_foot_transform_stamped.transform.translation.z);
    }
    catch (const tf2::TransformException &ex)
    {
      // RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
    // get_robot_state();

    // Example publishing logic (can be customized)
    // std_msgs::msg::Int8 message = std_msgs::msg::Int8();
    // message.data = 0; // Example command
    // publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_;
  rclcpp::Publisher<shi2d2_interfaces::msg::FootPose>::SharedPtr foot_pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Transform body_transform_;
  nav_msgs::msg::Odometry body_odometry_;
  sensor_msgs::msg::Imu body_imu_data_;
  geometry_msgs::msg::Pose body_pose_;

  BodyState body_state_; 



  Eigen::MatrixXd Px_;
  Eigen::MatrixXd PuPuRQIPuT_;

  std::vector<double> t_;
  std::vector<double> zmp_refs_x_;
  std::vector<double> zmp_refs_y_;
  std::vector<geometry_msgs::msg::Vector3> left_footstep_positions_;
  std::vector<geometry_msgs::msg::Vector3> right_footstep_positions_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionPlanner>());
  rclcpp::shutdown();
  return 0;
}
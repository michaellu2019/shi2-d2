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
  LocomotionPlanner() : Node("locomotion_planner"), tick_count_(0)
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

    // init_zmp_mpc();
    // solve_zmp_mpc();
    mpc();
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

  static double mpc_cost_function(unsigned n, const double *u, double *grad, void *data)
  {
    MPCData *mpc_data = static_cast<MPCData *>(data);
    // const Eigen::Vector3d &X = mpc_data->X;
    double x = mpc_data->X[0];
    double v = mpc_data->X[1];
    double a = mpc_data->X[2];
    const std::vector<double> &v_refs = mpc_data->v_refs;
    
    double cost = 0.0;
    int k = 0;

    for (int i = k; i < k + ZMP_MPC_NUM_TIMESTEPS; i++) {
      a = g/ZMP_MPC_COM_HEIGHT_M * (u[i]);
      // a = g/ZMP_MPC_COM_HEIGHT_M * (x - u[i]);
      v = v + a * ZMP_MPC_TIMESTEP_DURATION_SEC;
      x = x + v * ZMP_MPC_TIMESTEP_DURATION_SEC;
      double v_ref = v_refs[i + 1];
      
      if (grad) {
        // double grad1 = 2 * (u[i] - 1);
        double grad1 = 2 * MPC_V_PENALTY * (v - v_ref) * ZMP_MPC_TIMESTEP_DURATION_SEC * (-g/ZMP_MPC_COM_HEIGHT_M);
        // double grad2 = -2 * MPC_U_PENALTY * 1/pow(u[i], 3);
        double grad2 = 2 * MPC_U_PENALTY * u[i];
        grad[i] = grad1 + grad2;
      }

      // double iteration_cost1 = pow(u[i] - 1, 2);
      double iteration_cost1 = MPC_V_PENALTY * pow(v - v_ref, 2);
      // double iteration_cost2 = MPC_U_PENALTY * 1/pow(u[i], 2);
      double iteration_cost2 = MPC_U_PENALTY * pow(u[i], 2);
      cost += iteration_cost1 + iteration_cost2;
      // std::cout << i << "cost: " << iteration_cost << ", v_next: " << v_next << ", v_ref_next: " << v_ref_next << std::endl;
    }


    return cost;
  }

  static void mpc_multi_constraint(unsigned m, double *result, unsigned n, const double *u, double *grad, void *data)
  {
    MPCData *mpc_data = static_cast<MPCData *>(data);
    double x = mpc_data->X[0];
    double v = mpc_data->X[1];
    double a = mpc_data->X[2];

    double step_period_ms = mpc_data->step_period_ms;
    double half_step_period_ms = step_period_ms/2.0;

    int k = 0;

    for (int i = k; i < k + ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = i * ZMP_MPC_TIMESTEP_DURATION_MS;
      bool left_foot_on_ground = t_ms % (int) step_period_ms < half_step_period_ms;

      if (grad) {
        grad[i] = left_foot_on_ground ? 1.0 : -1.0;
        // grad[i] = 1.0;
      }
      
      // result[i] = left_foot_on_ground ? -(x - u[i]) + 60e-3 : (x - u[i]) + 60e-3;
      // result[i] = left_foot_on_ground ? u[i] + 2 : -u[i] + 2;
      result[i] = left_foot_on_ground ? u[i] + 1.0 : -u[i] + 1.0;

      std::cout << t_ms << ": " << left_foot_on_ground << " -> " << x << " vs " << u[i] << " = " << result[i] << std::endl;
      
      a = g/ZMP_MPC_COM_HEIGHT_M * (x - u[i]);
      v = v + a * ZMP_MPC_TIMESTEP_DURATION_SEC;
      x = x + v * ZMP_MPC_TIMESTEP_DURATION_SEC;
    }

    return;
  }

  void mpc()
  {
    const double V_REF_X = 400e-3;
    const double V_REF_Y = 0.0;
    
    std::vector<double> v_refs_x(ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS, V_REF_X);
    std::vector<double> v_refs_y(ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS, V_REF_Y);

    double half_step_period_ms = ZMP_STEP_LENGTH_M/V_REF_X * 1000.0;
    double step_period_ms = half_step_period_ms * 2;
    double double_support_duration_ms = half_step_period_ms/5;
    double single_support_duration_ms = half_step_period_ms - double_support_duration_ms;
    double step_support_duration_ms = 2 * double_support_duration_ms + single_support_duration_ms;

    Eigen::Vector3d X(body_state_.x, body_state_.vx, body_state_.ax);
    Eigen::Vector3d Y(body_state_.y, body_state_.vy, body_state_.ay);
    std::vector<double> u_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> u_y(ZMP_MPC_NUM_TIMESTEPS);



    int k = 0;
    std::vector<double> u_y_lb;
    std::vector<double> u_y_ub;


    for (int i = k; i < k + ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = i * ZMP_MPC_TIMESTEP_DURATION_MS;
      bool left_foot_on_ground = t_ms % (int) step_period_ms < half_step_period_ms;

      if (left_foot_on_ground) {
        u_y_lb.push_back(-HUGE_VAL);
        u_y_ub.push_back(-MPC_FOOTSTEP_Y_INNER_BOUND);
        u_y[i] = -MPC_FOOTSTEP_Y_INNER_BOUND * 2;
      } else {
        u_y_lb.push_back(MPC_FOOTSTEP_Y_INNER_BOUND);
        u_y_ub.push_back(HUGE_VAL);
        u_y[i] = MPC_FOOTSTEP_Y_INNER_BOUND * 2;
      }
    }

    // for (int i = 0; i < u_y_lb.size(); i++) {
    //   std::cout << u_y_lb[i] << ", ";
    // }
    // std::cout << std::endl;
    // for (int i = 0; i < u_y_ub.size(); i++) {
    //   std::cout << u_y_ub[i] << ", ";
    // }
    // std::cout << std::endl;

    MPCData mpc_y_data{Y, step_period_ms, v_refs_y};
    std::vector<double> y_tol_constraints(ZMP_MPC_NUM_TIMESTEPS, 1e-8);

    // nlopt::opt opt_x(nlopt::LD_MMA, ZMP_MPC_NUM_TIMESTEPS);
    // opt_x.set_min_objective(zmp_mpc_cost_function, &zmp_mpc_x_data);
    // opt_x.set_xtol_rel(1e-4);
    // opt_x.set_maxtime(10.0);
    
    nlopt::opt opt_y(nlopt::LD_MMA, ZMP_MPC_NUM_TIMESTEPS);
    opt_y.set_min_objective(mpc_cost_function, &mpc_y_data);
    // opt_y.add_inequality_mconstraint(mpc_multi_constraint, &mpc_y_data, y_tol_constraints);
    opt_y.set_lower_bounds(u_y_lb);
    opt_y.set_upper_bounds(u_y_ub);
    opt_y.set_xtol_rel(1e-4);
    opt_y.set_maxtime(10.0);
    
    auto y_start_time = std::chrono::high_resolution_clock::now(); // Start timer
    double min_y_cost;
    try {
      nlopt::result result = opt_y.optimize(u_y, min_y_cost);
      
      auto y_end_time = std::chrono::high_resolution_clock::now(); // End timer
      auto y_duration = std::chrono::duration_cast<std::chrono::milliseconds>(y_end_time - y_start_time).count();
      RCLCPP_INFO(this->get_logger(), "Y Optimization succeeded. Min cost: %.3f", min_y_cost);
      RCLCPP_INFO(this->get_logger(), "Solver took %ld ms\n", y_duration);
      std::cout << std::endl;
    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "NLOPT failed: %s", e.what());
    }

    std::cout << "Footstep Values" << std::endl;
    std::vector<double> time;
    k = 0;
    for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      time.push_back(t_ms);
      std::cout << "(" << t_ms << ", " << u_x[i - k] << ", " << u_y[i - k] << "), ";
    }
    std::cout << std::endl;

    std::vector<double> com_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> com_v_x(ZMP_MPC_NUM_TIMESTEPS);

    double y_sim = Y[0];
    double v_y_sim = Y[1];
    double a_y_sim = Y[2];
    std::vector<double> com_y;
    std::vector<double> com_v_y;
    for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;

      com_y.push_back(y_sim);
      com_v_y.push_back(v_y_sim);

      std::cout << "Y[" << t_ms << "] = " << y_sim << ", "
                << "VY[" << t_ms << "] = " << v_y_sim << ", "
                << "AY[" << t_ms << "] = " << a_y_sim << ", "
                << "UY[" << t_ms << "] = " << u_y[i] << ", " << std::endl;
      
      a_y_sim = g/ZMP_MPC_COM_HEIGHT_M * (u_y[i]);
      // a_y_sim = g/ZMP_MPC_COM_HEIGHT_M * (y_sim - u_y[i]);
      v_y_sim = v_y_sim + a_y_sim * ZMP_MPC_TIMESTEP_DURATION_SEC;
      y_sim = y_sim + v_y_sim * ZMP_MPC_TIMESTEP_DURATION_SEC;

    }

    plot_mpc_results(time, com_x, com_v_x, com_y, com_v_y, u_x, u_y);
  }

  void plot_mpc_results(const std::vector<double>& time,  
                        const std::vector<double>& com_x, const std::vector<double>& com_v_x,
                        const std::vector<double>& com_y, const std::vector<double>& com_v_y,
                        const std::vector<double>& u_x, const std::vector<double>& u_y) {
    plt::figure();

    // Plot 1: Y vs X
    plt::subplot(5, 1, 1); // 3 rows, 1 column, 1st plot
    plt::plot(com_x, com_y, {{"label", "com_y vs com_x"}});
    plt::plot(u_x, u_y, {{"label", "u_y vs u_x"}});
    plt::xlabel("X (meters)");
    plt::ylabel("Y (meters)");
    plt::title("Trajectories in X-Y Plane");
    plt::legend();
    plt::grid(true);

    // Plot 2: X vs t
    plt::subplot(5, 1, 2); // 3 rows, 1 column, 2nd plot
    // plt::plot(time, com_x, {{"label", "COM X vs t"}});
    plt::plot(time, u_x, {{"label", "Ux vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("X (meters)");
    plt::title("Trajectory X over Time");
    plt::legend();
    plt::grid(true);
    
    // Plot 3: Y vs t
    plt::subplot(5, 1, 3); // 3 rows, 1 column, 3rd plot
    // plt::plot(time, com_y, {{"label", "COM Y vs t"}});
    plt::plot(time, u_y, {{"label", "Uy vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("Y (meters)");
    plt::title("Trajectory Y over Time");
    plt::legend();
    plt::grid(true);

    // Plot 4: Vx vs t
    plt::subplot(5, 1, 4); // 3 rows, 1 column, 4th plot
    plt::plot(time, com_v_x, {{"label", "Vx vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("Vx (m/s)");
    plt::title("Trajectory Vx over Time");
    plt::legend();
    plt::grid(true);

    // Plot 5: Vy vs t
    plt::subplot(5, 1, 5); // 3 rows, 1 column, 5th plot
    plt::plot(time, com_v_y, {{"label", "Vy vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("Vy (m/s)");
    plt::title("Trajectory Vy over Time");
    plt::legend();
    plt::grid(true);

    // // Plot 2: states
    // plt::subplot(4, 1, 4); // 3 rows, 1 column, 2nd plot
    // plt::plot(time, double_support, {{"label", "DSP vs t"}});
    // plt::plot(time, right_foot_down, {{"label", "RFD vs t"}});
    // plt::xlabel("Time (seconds)");
    // plt::ylabel("States");
    // plt::title("Gait States over Time");
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

  void plan_footsteps()
  {
    t_.clear();
    left_footstep_positions_.clear();
    right_footstep_positions_.clear();
    zmp_refs_x_.clear();
    zmp_refs_y_.clear();

    // generate footsteps and zmp reference positions
    for (int i = 0; i < ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      int t_walk_ms = std::max(0, (int) (t_ms - ZMP_MPC_NUM_STATIONARY_FOOTSTEP_PLANNING_TIMESTEPS * ZMP_MPC_TIMESTEP_DURATION_MS));
      t_.push_back(t_ms);

      geometry_msgs::msg::Vector3 left_foot_position;
      geometry_msgs::msg::Vector3 right_foot_position;

      if (i < ZMP_MPC_NUM_STATIONARY_FOOTSTEP_PLANNING_TIMESTEPS) {
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
      } else if (i > ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS - ZMP_MPC_NUM_STATIONARY_FOOTSTEP_PLANNING_TIMESTEPS) {
        double last_step_x = body_state_.x + ((ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS - ZMP_MPC_NUM_STATIONARY_FOOTSTEP_PLANNING_TIMESTEPS) * ZMP_MPC_TIMESTEP_DURATION_MS)/step_period_ms * ZMP_STEP_LENGTH_M;

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

  void solve_zmp_mpc(int k)
  {
    // get_robot_state(); ??????

    // solve optimal control (jerk values) for the ZMP MPC problem
    auto zmp_mpc_start = std::chrono::high_resolution_clock::now(); // Start timer
    
    Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_X;
    Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_Y;
    for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int j = k + i;
      if (j < zmp_refs_x_.size() && j < zmp_refs_y_.size()) {
        ZMP_REFS_X(i) = zmp_refs_x_[j];
        ZMP_REFS_Y(i) = zmp_refs_y_[j];
      } else {
        ZMP_REFS_X(i) = 0.0; // pad with zeroes
        ZMP_REFS_Y(i) = 0.0; // pad with zeroes
      }
    }
    
    // Eigen::Vector3d X(body_state_.x, body_state_.vx, body_state_.ax);
    // Eigen::Vector3d Y(body_state_.y, body_state_.vy, body_state_.ay);
    Eigen::Vector3d X(-body_state_.x, -body_state_.vx, -body_state_.ax);
    Eigen::Vector3d Y(-body_state_.y, -body_state_.vy, -body_state_.ay);
    // Eigen::Vector3d X(0, 0, 0);
    // Eigen::Vector3d Y(0, 0, 0);
    
    // Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_X(zmp_refs_x_.data()); 
    // Eigen::Vector<double, ZMP_MPC_NUM_TIMESTEPS> ZMP_REFS_Y(zmp_refs_y_.data());

    // Px_ - N x 3
    // X, Y - 3 x 1
    // ZMP_REFS_X, ZMP_REFS_Y - N x 1
    // PuPuRQIPuT_ - N x N
    // PxZ, PyZ - N x 1
    // Ux, Uy - N x 1

    // print sizes of Px_, X, Y, and ZMP_REFS_X and ZMP_REFS_Y
    // std::cout << "Px_ size: " << Px_.rows() << " x " << Px_.cols() << std::endl;
    // std::cout << "X size: " << X.size() << std::endl;
    // std::cout << "Y size: " << Y.size() << std::endl;
    // std::cout << "ZMP_REFS_X size: " << ZMP_REFS_X.size() << std::endl;
    // std::cout << "ZMP_REFS_Y size: " << ZMP_REFS_Y.size() << std::endl;
    // std::cout << "PuPuRQIPuT_ size: " << PuPuRQIPuT_.rows() << " x " << PuPuRQIPuT_.cols() << std::endl;

    Eigen::VectorXd PxZMP = Px_ * X - ZMP_REFS_X;
    Eigen::VectorXd PyZMP = Px_ * Y - ZMP_REFS_Y;

    Eigen::VectorXd Ux = PuPuRQIPuT_ * PxZMP;
    Eigen::VectorXd Uy = PuPuRQIPuT_ * PyZMP;

    // std::cout << "Px:\n" << Px_ << std::endl;
    // std::cout << "X:\n" << X << std::endl;
    // std::cout << "Y:\n" << Y << std::endl;
    // std::cout << "ZMP_REFS_X:\n" << ZMP_REFS_X << std::endl;
    // std::cout << "ZMP_REFS_Y:\n" << ZMP_REFS_Y << std::endl;
    // std::cout << "PuPuRQIPuT:\n" << PuPuRQIPuT_ << std::endl;
    // std::cout << "Ux:\n" << Ux << std::endl;
    // std::cout << "Uy:\n" << Uy << std::endl;

    auto zmp_mpc_end = std::chrono::high_resolution_clock::now(); // End timer
    auto zmp_mpc_duration = std::chrono::duration_cast<std::chrono::milliseconds>(zmp_mpc_end - zmp_mpc_start).count();
    // std::cout << "Solving ZMP MPC for k=" << k << " took " << zmp_mpc_duration << " ms" << std::endl;
    // std::cout << "ZMP REF X=" << ZMP_REFS_X[0] << ", Y=" << ZMP_REFS_Y[0] << std::endl;
    // std::cout << "X=" << X[0] << ", " << X[1] << ", " << X[2] << std::endl;
    // std::cout << "Y=" << Y[0] << ", " << Y[1] << ", " << Y[2] << std::endl;
    // std::cout << "Ux=" << Ux[0] << ", Uy=" << Uy[0] << std::endl;
    
    // integrate COM trajectory forward in time
    // calculate ZMP values based on COM values
    // std::cout << "ZMP Values" << std::endl;
    std::vector<shi2d2_interfaces::msg::FootPose> left_foot_poses;
    std::vector<shi2d2_interfaces::msg::FootPose> right_foot_poses;
    std::vector<double> com_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> com_y(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_y(ZMP_MPC_NUM_TIMESTEPS);
    Eigen::Vector3d Xp(X[0], X[1], X[2]);
    Eigen::Vector3d Yp(Y[0], Y[1], Y[2]);

    // std::vector<double> COM_X = {-0.0199548, -0.0199505, -0.0199395, -0.0199191, -0.0198874, -0.0198426, -0.0197835, -0.019709, -0.0196185, -0.0195113, -0.0193871, -0.0192458, -0.0190873, -0.0189116, -0.0187189, -0.0185093, -0.0182832, -0.0180407, -0.0177824, -0.0175084, -0.0172192, -0.0169152, -0.0165968, -0.0162643, -0.0159181, -0.0155586, -0.0151861, -0.0148011, -0.0144037, -0.0139944, -0.0135735, -0.0131411, -0.0126976, -0.0122431, -0.0117779, -0.0113022, -0.0108161, -0.0103198, -0.00981336, -0.00929688, -0.00877044, -0.00823408, -0.00768783, -0.0071317, -0.00656566, -0.00598967, -0.00540368, -0.00480759, -0.00420132, -0.00358474, -0.0029577, -0.00232006, -0.00167164, -0.00101225, -0.000341674, 0.000340296, 0.0010339, 0.00173938, 0.002457, 0.00318701, 0.0039297, 0.00468534, 0.00545422, 0.0062366, 0.00703277, 0.007843, 0.00866754, 0.00950665, 0.0103605, 0.0112294, 0.0121134, 0.0130126, 0.0139272, 0.014857, 0.0158021, 0.0167623, 0.0177373, 0.0187267, 0.01973, 0.0207465, 0.0217752, 0.0228151, 0.0238647, 0.0249226, 0.0259872, 0.0270573, 0.0281316, 0.0292092, 0.0302895, 0.0313719, 0.0324559, 0.0335413, 0.0346278, 0.0357153, 0.0368037, 0.0378929, 0.0389829, 0.0400736, 0.0411648, 0.0422565, 0.0433485, 0.0444404, 0.0455318, 0.0466223, 0.0477116, 0.0487991, 0.0498847, 0.0509683, 0.0520499, 0.0531294, 0.0542071, 0.0552831, 0.0563576, 0.0574309, 0.058503, 0.0595743, 0.060645, 0.0617151, 0.0627847, 0.063854, 0.0649228, 0.065991, 0.0670583, 0.0681243, 0.0691887, 0.0702512, 0.0713117, 0.0723701, 0.0734265, 0.0744808, 0.0755334, 0.0765844, 0.077634, 0.0786826, 0.0797302, 0.0807771, 0.0818237, 0.0828699, 0.0839159, 0.0849619, 0.0860076, 0.0870529, 0.0880976, 0.0891413, 0.0901838, 0.0912246, 0.0922637, 0.093301, 0.0943365, 0.0953704, 0.0964027, 0.0974338, 0.0984638, 0.0994929, 0.100521, 0.10155, 0.102578, 0.103606, 0.104634, 0.105662, 0.10669, 0.107718, 0.108746, 0.109773, 0.110799, 0.111824, 0.112847, 0.113869, 0.114889, 0.115907, 0.116924, 0.117941, 0.118956, 0.119971, 0.120985, 0.121999, 0.123013, 0.124028, 0.125042, 0.126057, 0.127072, 0.128087, 0.129102, 0.130116, 0.13113, 0.132142, 0.133152, 0.134162, 0.13517, 0.136176, 0.137181, 0.138186, 0.139189, 0.140192, 0.141195, 0.142197, 0.1432, 0.144203, 0.145206, 0.146209, 0.147213, 0.148217, 0.14922, 0.150223, 0.151225, 0.152226, 0.153225, 0.154223, 0.155219, 0.156214, 0.157208, 0.158201, 0.159193, 0.160184, 0.161175, 0.162165, 0.163156, 0.164147, 0.165137, 0.166129, 0.16712, 0.168111, 0.169101, 0.170091, 0.17108, 0.172068, 0.173053, 0.174037, 0.17502, 0.176001, 0.17698, 0.177958, 0.178935, 0.179911, 0.180886, 0.181861, 0.182835, 0.183809, 0.184783, 0.185757, 0.18673, 0.187703, 0.188675, 0.189646, 0.190616, 0.191583, 0.192549, 0.193512, 0.194473, 0.195432, 0.196389, 0.197344, 0.198297, 0.199248, 0.200199, 0.201148, 0.202096, 0.203043, 0.203989, 0.204934, 0.205878, 0.206821, 0.207762, 0.208701, 0.209638, 0.210572, 0.211502, 0.21243, 0.213354, 0.214275, 0.215193, 0.216108, 0.21702, 0.217929, 0.218836, 0.21974, 0.220642, 0.221542, 0.222439, 0.223334, 0.224227, 0.225117, 0.226004, 0.226887, 0.227766, 0.228641, 0.229511, 0.230377, 0.231238, 0.232094, 0.232945, 0.233792, 0.234634, 0.235473, 0.236307, 0.237138, 0.237965, 0.238789, 0.23961, 0.240427};
    // std::vector<double> COM_Y = {4.22747e-06, 2.48749e-06, -2.03949e-06, -1.04498e-05, -2.3664e-05, -4.24517e-05, -6.74525e-05, -9.91947e-05, -0.000138112, -0.000184557, -0.000238813, -0.000301106, -0.000371615, -0.000450474, -0.000537788, -0.000633632, -0.000738057, -0.000851099, -0.000972778, -0.0011031, -0.00124207, -0.00138968, -0.00154592, -0.00171077, -0.00188423, -0.00206627, -0.00225688, -0.00245604, -0.00266374, -0.00287995, -0.00310466, -0.00333784, -0.00357947, -0.00382953, -0.00408797, -0.00435476, -0.00462983, -0.00491311, -0.00520452, -0.00550395, -0.00581126, -0.00612628, -0.00644881, -0.00677858, -0.0071153, -0.00745859, -0.00780801, -0.00816303, -0.008523, -0.00888719, -0.0092547, -0.00962448, -0.00999533, -0.0103658, -0.0107342, -0.0110985, -0.0114566, -0.0118058, -0.0121429, -0.0124644, -0.0127663, -0.0130438, -0.0132914, -0.0135035, -0.0136745, -0.0137993, -0.0138737, -0.0138944, -0.0138587, -0.0137651, -0.0136123, -0.0134001, -0.0131289, -0.0127997, -0.0124145, -0.0119758, -0.0114872, -0.010953, -0.0103788, -0.00977096, -0.00913743, -0.00848741, -0.00783158, -0.00718135, -0.00654811, -0.00594236, -0.00537303, -0.00484752, -0.00437185, -0.00395072, -0.00358771, -0.00328526, -0.00304477, -0.00286664, -0.00275026, -0.00269398, -0.00269517, -0.00275005, -0.00285373, -0.00300004, -0.00318145, -0.00338894, -0.00361195, -0.0038391, -0.00405902, -0.00426116, -0.00443648, -0.00457747, -0.00467796, -0.00473305, -0.00473898, -0.00469309, -0.00459374, -0.00444031, -0.00423315, -0.00397365, -0.00366421, -0.00330831, -0.00291061, -0.002477, -0.00201475, -0.00153262, -0.00104091, -0.000550724, -7.31955e-05, 0.000381391, 0.000804251, 0.00118811, 0.00152706, 0.00181642, 0.0020527, 0.00223346, 0.00235729, 0.0024238, 0.00243359, 0.00238824, 0.00229039, 0.00214374, 0.00195317, 0.00172476, 0.00146601, 0.00118589, 0.000894901, 0.000604359, 0.000325592, 6.90919e-05, -0.000156157, -0.00034268, -0.000484367, -0.000576355, -0.000614932, -0.000597471, -0.000522375, -0.000389046, -0.000197882, 4.97186e-05, 0.000351328, 0.000703437, 0.00110138, 0.00153926, 0.00200979, 0.0025042, 0.0030122, 0.00352268, 0.00402452, 0.00450744, 0.00496268, 0.00538297, 0.00576244, 0.00609643, 0.00638148, 0.00661517, 0.00679616, 0.00692408, 0.00699957, 0.00702429, 0.00700092, 0.00693322, 0.00682612, 0.0066858, 0.0065198, 0.0063372, 0.00614856, 0.0059653, 0.00579883, 0.00565975, 0.00555713, 0.00549856, 0.00549027, 0.00553724, 0.00564329, 0.00581119, 0.00604268, 0.00633847, 0.00669834, 0.00712102, 0.00760426, 0.0081447, 0.00873786, 0.009378, 0.010058, 0.0107694, 0.011502, 0.0122449, 0.0129872, 0.0137189, 0.0144314, 0.0151177, 0.0157722, 0.0163904, 0.0169692, 0.0175065, 0.0180011, 0.018453, 0.0188632, 0.0192336, 0.0195671, 0.0198681, 0.0201416, 0.0203942, 0.0206339, 0.02087, 0.0211137, 0.0213766, 0.0216707, 0.0220069, 0.0223947, 0.0228424, 0.0233564, 0.0239424, 0.0246047, 0.0253465, 0.0261701, 0.0270769, 0.0280672, 0.0291402, 0.0302945, 0.0315272, 0.0328345, 0.0342115, 0.0356517, 0.0371472, 0.0386888, 0.0402662, 0.0418694, 0.0434892, 0.0451178, 0.0467492, 0.0483785, 0.0500024, 0.0516185, 0.0532258, 0.0548242, 0.0564146, 0.0579992, 0.0595809, 0.0611639, 0.0627535, 0.0643562, 0.0659798, 0.0676334, 0.0693278, 0.0710754, 0.0728894, 0.0747829, 0.0767685, 0.0788571, 0.0810585, 0.0833807, 0.0858308, 0.0884148, 0.0911377, 0.0940032, 0.0970144, 0.100173, 0.103481, 0.106937, 0.110541, 0.114291, 0.118183, 0.122212, 0.126372, 0.130655, 0.135053, 0.139556, 0.144156, 0.148847, 0.153624, 0.158481, 0.163416, 0.168427, 0.173511, 0.178668, 0.183897, 0.189198, 0.194571, 0.200016, 0.205534, 0.211125, 0.21679};
    
    std::vector<double> COM_X = {0.0199296, 0.0199287, 0.0199265, 0.0199225, 0.0199165, 0.0199082, 0.0198976, 0.0198848, 0.0198698, 0.0198528, 0.019834, 0.0198136, 0.0197919, 0.0197692, 0.0197459, 0.0197222, 0.0196986, 0.0196755, 0.0196532, 0.019632, 0.0196125, 0.0195949, 0.0195798, 0.0195674, 0.0195583, 0.0195527, 0.0195511, 0.0195538, 0.0195613, 0.0195739, 0.0195921, 0.0196161, 0.0196464, 0.0196834, 0.0197274, 0.0197788, 0.019838, 0.0199053, 0.0199811, 0.0200658, 0.0201597, 0.0202633, 0.0203768, 0.0205007, 0.0206353, 0.020781, 0.0209382, 0.0211072, 0.0212884, 0.0214822, 0.021689, 0.0219091, 0.022143, 0.0223911, 0.0226536, 0.0229311, 0.0232238, 0.0235323, 0.0238568, 0.0241978, 0.0245556, 0.0249307, 0.0253233, 0.0257339, 0.0261629, 0.0266104, 0.0270769, 0.0275627, 0.0280679, 0.0285929, 0.0291378, 0.0297028, 0.0302879, 0.0308931, 0.0315185, 0.0321638, 0.0328287, 0.0335129, 0.0342158, 0.0349367, 0.0356746, 0.0364285, 0.0371969, 0.0379782, 0.0387711, 0.0395739, 0.0403857, 0.0412054, 0.0420322, 0.0428657, 0.0437053, 0.0445507, 0.0454016, 0.0462578, 0.0471192, 0.0479857, 0.0488571, 0.0497333, 0.0506142, 0.0514996, 0.0523893, 0.0532828, 0.0541797, 0.0550796, 0.0559819, 0.0568862, 0.0577923, 0.0587, 0.0596092, 0.06052, 0.0614323, 0.0623463, 0.0632622, 0.06418, 0.0651001, 0.0660224, 0.0669473, 0.0678747, 0.0688047, 0.0697373, 0.0706723, 0.0716096, 0.0725488, 0.0734895, 0.0744313, 0.075374, 0.0763172, 0.077261, 0.0782052, 0.07915, 0.0800954, 0.0810416, 0.0819889, 0.0829373, 0.0838871, 0.0848385, 0.0857916, 0.0867466, 0.0877036, 0.0886624, 0.0896231, 0.0905854, 0.091549, 0.0925136, 0.0934788, 0.0944442, 0.0954096, 0.0963751, 0.0973405, 0.098306, 0.0992716, 0.100238, 0.101204, 0.102171, 0.10314, 0.104109, 0.10508, 0.106052, 0.107026, 0.108001, 0.108977, 0.109955, 0.110934, 0.111914, 0.112894, 0.113873, 0.114853, 0.115832, 0.11681, 0.117789, 0.118767, 0.119745, 0.120724, 0.121703, 0.122683, 0.123663, 0.124645, 0.125628, 0.126612, 0.127597, 0.128584, 0.129571, 0.13056, 0.131549, 0.132537, 0.133526, 0.134514, 0.135501, 0.136488, 0.137474, 0.13846, 0.139446, 0.140432, 0.141419, 0.142406, 0.143393, 0.144381, 0.145371, 0.146361, 0.147353, 0.148346, 0.149339, 0.150334, 0.151328, 0.152322, 0.153316, 0.154309, 0.155302, 0.156294, 0.157285, 0.158276, 0.159267, 0.160257, 0.161248, 0.162239, 0.163231, 0.164223, 0.165217, 0.166211, 0.167206, 0.168203, 0.1692, 0.170198, 0.171196, 0.172193, 0.17319, 0.174187, 0.175183, 0.176177, 0.177172, 0.178165, 0.179159, 0.180152, 0.181146, 0.182139, 0.183134, 0.184128, 0.185124, 0.186121, 0.187119, 0.188118, 0.189117, 0.190117, 0.191117, 0.192116, 0.193116, 0.194114, 0.195111, 0.196108, 0.197104, 0.1981, 0.199095, 0.20009, 0.201085, 0.20208, 0.203076, 0.204072, 0.205069, 0.206068, 0.207067, 0.208067, 0.209068, 0.210069, 0.21107, 0.212071, 0.213071, 0.21407, 0.215069, 0.216067, 0.217064, 0.21806, 0.219057, 0.220052, 0.221048, 0.222045, 0.223041, 0.224039, 0.225037, 0.226036, 0.227036, 0.228037, 0.229038, 0.23004, 0.231042, 0.232043, 0.233044, 0.234045, 0.235044, 0.236042, 0.23704, 0.238037, 0.239034, 0.24003, 0.241027, 0.242024, 0.243021, 0.244019, 0.245017, 0.246017, 0.247017, 0.248019, 0.249021, 0.250023, 0.251025, 0.252027, 0.253029, 0.254029, 0.255029, 0.256028, 0.257026, 0.258023, 0.25902, 0.260017, 0.261014, 0.262011, 0.263008, 0.264007, 0.265006, 0.266005, 0.267006, 0.268008, 0.26901, 0.270012, 0.271015, 0.272017, 0.273019, 0.274019, 0.275019, 0.276018, 0.277017, 0.278014, 0.279012, 0.280009, 0.281006, 0.282003, 0.283, 0.283999, 0.284998, 0.285998, 0.286999, 0.288, 0.289003, 0.290005, 0.291008, 0.29201, 0.293012, 0.294013, 0.295013, 0.296012, 0.29701, 0.298008, 0.299005, 0.300002, 0.301, 0.301997, 0.302995, 0.303993, 0.304992, 0.305992, 0.306993, 0.307995, 0.308997, 0.31, 0.311002, 0.312005, 0.313007, 0.314007, 0.315007, 0.316007, 0.317005, 0.318003, 0.319, 0.319997, 0.320994, 0.321991, 0.322989, 0.323987, 0.324986, 0.325986, 0.326987, 0.327989, 0.328991, 0.329994, 0.330997, 0.331999, 0.333001, 0.334001, 0.335001, 0.336, 0.336999, 0.337996, 0.338993, 0.33999, 0.340987, 0.341984, 0.342982, 0.34398, 0.344979, 0.345979, 0.34698, 0.347981, 0.348983, 0.349986, 0.350988, 0.351991, 0.352992, 0.353993, 0.354992, 0.355991, 0.356989, 0.357987, 0.358984, 0.35998, 0.360977, 0.361974, 0.362971, 0.363969, 0.364968, 0.365967, 0.366968, 0.367969, 0.368971, 0.369973, 0.370975, 0.371977, 0.372978, 0.373978, 0.374978, 0.375976, 0.376974, 0.377971, 0.378967, 0.379963, 0.38096, 0.381956, 0.382953, 0.38395, 0.384948, 0.385947, 0.386947, 0.387948, 0.388949, 0.389951, 0.390952, 0.391953, 0.392954, 0.393953, 0.394952, 0.395949, 0.396946, 0.397942, 0.398938, 0.399934, 0.400929, 0.401924, 0.40292, 0.403917, 0.404914, 0.405912, 0.406911, 0.40791, 0.40891, 0.409911, 0.410911, 0.411911, 0.41291, 0.413908, 0.414905, 0.415902, 0.416897, 0.417892, 0.418886, 0.41988, 0.420874, 0.421868, 0.422862, 0.423856, 0.424852, 0.425848, 0.426845, 0.427842, 0.42884, 0.429839, 0.430837, 0.431834, 0.432831, 0.433827, 0.434822, 0.435816, 0.436808, 0.4378, 0.438792, 0.439783, 0.440774, 0.441764, 0.442756, 0.443747, 0.444739, 0.445732, 0.446725, 0.447719, 0.448713, 0.449708, 0.450702, 0.451695, 0.452688, 0.453679, 0.45467, 0.455659, 0.456647, 0.457634, 0.45862, 0.459606, 0.460591, 0.461577, 0.462562, 0.463547, 0.464533, 0.46552, 0.466507, 0.467494, 0.468481, 0.469468, 0.470455, 0.471441, 0.472426, 0.473409, 0.474391, 0.475372, 0.476351, 0.477329, 0.478306, 0.479282, 0.480257, 0.481232, 0.482207, 0.483182, 0.484156, 0.485131, 0.486106, 0.48708, 0.488055, 0.489029, 0.490002, 0.490974, 0.491944, 0.492913, 0.493879, 0.494844, 0.495807, 0.496768, 0.497727, 0.498686, 0.499642, 0.500598, 0.501553, 0.502507, 0.503461, 0.504414, 0.505367, 0.506318, 0.507269, 0.508219, 0.509167, 0.510113, 0.511056, 0.511997, 0.512935, 0.51387, 0.514803, 0.515733, 0.51666, 0.517585, 0.518508, 0.519428, 0.520347, 0.521264, 0.522179, 0.523093, 0.524004, 0.524914, 0.525822, 0.526727, 0.52763, 0.528529, 0.529424, 0.530316, 0.531203, 0.532087, 0.532966, 0.533842, 0.534714, 0.535582, 0.536447, 0.537308, 0.538167, 0.539023, 0.539877, 0.540728, 0.541578};
    std::vector<double> COM_Y = {-4.62303e-06, -6.39128e-06, -1.0981e-05, -1.94991e-05, -3.2875e-05, -5.18854e-05, -7.71762e-05, -0.000109281, -0.000148636, -0.000195598, -0.000250454, -0.000313432, -0.000384709, -0.000464425, -0.000552683, -0.000649558, -0.000755104, -0.000869354, -0.00099233, -0.00112404, -0.00126449, -0.00141366, -0.00157155, -0.00173814, -0.00191343, -0.00209739, -0.00229, -0.00249126, -0.00270113, -0.00291961, -0.00314668, -0.00338231, -0.00362648, -0.00387917, -0.00414033, -0.00440992, -0.00468789, -0.00497418, -0.00526868, -0.00557129, -0.00588189, -0.00620029, -0.00652629, -0.00685965, -0.00720005, -0.00754713, -0.00790045, -0.00825947, -0.00862356, -0.00899198, -0.00936384, -0.00973809, -0.0101135, -0.0104887, -0.0108619, -0.0112313, -0.0115944, -0.0119488, -0.0122913, -0.0126184, -0.012926, -0.0132092, -0.0134629, -0.0136811, -0.0138584, -0.0139897, -0.0140708, -0.0140983, -0.0140696, -0.0139832, -0.0138378, -0.0136332, -0.0133698, -0.0130487, -0.0126717, -0.0122415, -0.0117616, -0.0112364, -0.0106713, -0.010073, -0.00944916, -0.00880913, -0.00816355, -0.00752387, -0.00690147, -0.00630685, -0.00574898, -0.00523525, -0.00477168, -0.00436301, -0.0040128, -0.00372351, -0.00349657, -0.00333236, -0.00323029, -0.00318875, -0.00320508, -0.00327554, -0.00339524, -0.00355804, -0.00375642, -0.00398135, -0.00422231, -0.00446793, -0.00470685, -0.00492854, -0.00512398, -0.00528567, -0.00540746, -0.00548447, -0.00551296, -0.00549029, -0.00541483, -0.00528598, -0.00510414, -0.00487068, -0.00458805, -0.00425975, -0.00389045, -0.00348608, -0.00305392, -0.00260278, -0.00214296, -0.00168561, -0.00124188, -0.000822094, -0.000435059, -8.80845e-05, 0.000212887, 0.000463158, 0.00065918, 0.000798488, 0.000879645, 0.000902216, 0.000866755, 0.000774818, 0.000628993, 0.000432945, 0.000191495, -8.92933e-05, -0.000401992, -0.000737664, -0.00108587, -0.00143533, -0.00177477, -0.00209376, -0.00238337, -0.00263617, -0.00284612, -0.00300842, -0.00311941, -0.00317653, -0.00317826, -0.00312405, -0.00301439, -0.00285073, -0.00263559, -0.00237254, -0.00206634, -0.00172297, -0.00134978, -0.000955645, -0.000550939, -0.000146869, 0.000245356, 0.000615357, 0.000954266, 0.00125472, 0.00151073, 0.00171753, 0.00187154, 0.00197024, 0.00201215, 0.00199678, 0.00192465, 0.00179726, 0.00161718, 0.00138802, 0.00111456, 0.000802838, 0.000460238, 9.56652e-05, -0.000280472, -0.000656932, -0.00102247, -0.00136669, -0.00168067, -0.00195704, -0.00218977, -0.00237409, -0.00250636, -0.00258405, -0.00260565, -0.00257067, -0.00247958, -0.00233388, -0.00213609, -0.00188982, -0.00159984, -0.00127214, -0.000914106, -0.00053462, -0.000144074, 0.000246308, 0.000625299, 0.000982509, 0.00130905, 0.00159755, 0.001842, 0.00203763, 0.00218084, 0.00226909, 0.0023009, 0.00227577, 0.00219419, 0.00205769, 0.00186877, 0.00163108, 0.00134937, 0.00102967, 0.00067935, 0.000307314, -7.60416e-05, -0.000459482, -0.000831775, -0.00118252, -0.00150282, -0.00178531, -0.00202395, -0.00221398, -0.00235178, -0.00243483, -0.00246161, -0.00243164, -0.0023454, -0.00220439, -0.00201115, -0.00176928, -0.00148356, -0.00115998, -0.000805943, -0.000430324, -4.35235e-05, 0.00034323, 0.000718708, 0.00107251, 0.00139575, 0.00168106, 0.0019224, 0.00211503, 0.00225532, 0.00234075, 0.00236981, 0.00234202, 0.00225786, 0.00211884, 0.0019275, 0.00168744, 0.00140343, 0.00108149, 0.000729003, 0.000354854, -3.05558e-05, -0.000415997, -0.000790237, -0.00114288, -0.00146503, -0.00174931, -0.00198971, -0.00218146, -0.00232094, -0.00240562, -0.002434, -0.00240559, -0.00232088, -0.00218137, -0.0019896, -0.00174917, -0.00146485, -0.00114266, -0.000789985, -0.000415705, -3.02211e-05, 0.000355236, 0.000729434, 0.00108198, 0.00140397, 0.00168804, 0.00192817, 0.0021196, 0.00225869, 0.00234294, 0.00237082, 0.00234186, 0.00225654, 0.00211637, 0.00192387, 0.00168266, 0.0013975, 0.00107442, 0.00072078, 0.000345482, -4.10813e-05, -0.000427679, -0.00080308, -0.00115689, -0.00148022, -0.00176568, -0.00200727, -0.00220022, -0.00234091, -0.00242682, -0.00245645, -0.00242929, -0.00234585, -0.00220763, -0.00201716, -0.00177806, -0.00149509, -0.00117427, -0.000822987, -0.000450123, -6.60811e-05, 0.000317907, 0.000690609, 0.00104163, 0.00136207, 0.00164455, 0.00188306, 0.00207282, 0.00221022, 0.00229274, 0.00231885, 0.00228808, 0.00220091, 0.00205884, 0.0018644, 0.0016212, 0.001334, 0.00100883, 0.000653047, 0.000275552, -0.000113263, -0.00050217, -0.000879941, -0.00123618, -0.001562, -0.00185003, -0.00209425, -0.0022899, -0.00243337, -0.00252212, -0.00255467, -0.00253053, -0.00245018, -0.00231513, -0.00212792, -0.00189217, -0.00161265, -0.00129538, -0.000947731, -0.000578611, -0.00019842, 0.000181609, 0.00055024, 0.000897069, 0.0012132, 0.00149126, 0.00172521, 0.00191028, 0.00204287, 0.00212042, 0.00214143, 0.00210541, 0.00201284, 0.00186521, 0.00166505, 0.00141596, 0.00112271, 0.000791301, 0.000429109, 4.50134e-05, -0.000350595, -0.000746494, -0.00113146, -0.00149511, -0.00182856, -0.00212443, -0.00237673, -0.00258069, -0.00273272, -0.00283028, -0.0028719, -0.00285709, -0.00278635, -0.0026612, -0.00248418, -0.00225892, -0.00199019, -0.00168403, -0.00134783, -0.000990497, -0.000622437, -0.000254899, 0.00010087, 0.000434457, 0.000736956, 0.00100097, 0.00122046, 0.00139065, 0.00150791, 0.00156968, 0.00157444, 0.00152168, 0.00141187, 0.00124649, 0.00102805, 0.000760141, 0.000447506, 9.61358e-05, -0.000286615, -0.000691882, -0.0011093, -0.00152765, -0.00193575, -0.00232321, -0.00268119, -0.00300233, -0.00328065, -0.00351142, -0.00369105, -0.00381705, -0.00388796, -0.00390331, -0.00386364, -0.00377049, -0.00362644, -0.00343513, -0.00320138, -0.00293125, -0.00263217, -0.00231306, -0.00198438, -0.00165741, -0.00134343, -0.0010529, -0.000794747, -0.000576422, -0.000403994, -0.00028229, -0.000214988, -0.000204683, -0.000252939, -0.000360322, -0.000526404, -0.00074976, -0.00102793, -0.00135738, -0.00173342, -0.00215012, -0.00260019, -0.00307481, -0.00356368, -0.00405566, -0.00453962, -0.00500526, -0.00544378, -0.00584792, -0.00621176, -0.00653065, -0.00680109, -0.00702066, -0.007188, -0.00730271, -0.00736544, -0.00737781, -0.0073425, -0.00726325, -0.00714499, -0.00699387, -0.00681743, -0.00662473, -0.00642632, -0.00623361, -0.006058, -0.00591007, -0.0057989, -0.00573205, -0.00571576, -0.00575497, -0.00585353, -0.00601418, -0.00623864, -0.00652764, -0.00688093, -0.00729724, -0.00777431, -0.00830879, -0.00889616, -0.00953071, -0.0102053, -0.0109114, -0.0116389, -0.0123769, -0.0131144, -0.0138415, -0.0145495, -0.0152315, -0.0158817, -0.0164959, -0.0170707, -0.0176041, -0.018095, -0.0185433, -0.0189499, -0.0193169, -0.0196471, -0.0199448, -0.0202152, -0.0204647, -0.0207015, -0.0209347, -0.0211756, -0.0214357, -0.0217271, -0.0220607, -0.022446, -0.0228912, -0.0234028, -0.0239864, -0.0246464, -0.0253859, -0.0262073, -0.027112, -0.0281001, -0.029171, -0.0303232, -0.0315539, -0.0328593, -0.0342343, -0.0356726, -0.0371662, -0.0387059, -0.0402815, -0.0418829, -0.0435009, -0.0451277, -0.0467573, -0.0483849, -0.0500071, -0.0516215, -0.053227, -0.0548237, -0.0564124, -0.0579952, -0.0595752, -0.0611566, -0.0627445, -0.0643455, -0.0659673, -0.0676192, -0.0693119, -0.0710578, -0.07287, -0.0747617, -0.0767455, -0.0788324, -0.0810319, -0.0833522, -0.0858005, -0.0883827, -0.0911036, -0.0939672, -0.0969764, -0.100133, -0.103439, -0.106893, -0.110495, -0.114243, -0.118132, -0.122159, -0.126317, -0.130598, -0.134993, -0.139493, -0.144091, -0.14878, -0.153554, -0.158408, -0.163341, -0.168349, -0.17343, -0.178584, -0.18381, -0.189108, -0.194478, -0.19992, -0.205434, -0.211022, -0.216683};

    int L = ZMP_MPC_NUM_TIMESTEPS;
    // int L = 1;
    
    // change to only use first element of Ux and Uy 
    // only use first com_x and com_y
    // only publish first left_foot_pose and right_foot_pose

    for (int i = 0; i < L; i++) {
      // std::cout << "Solving ZMP MPC for i=" << i << " took " << zmp_mpc_duration << " ms" << std::endl;
      // std::cout << "ZMP REF X=" << ZMP_REFS_X[i] << ", Y=" << ZMP_REFS_Y[i] << std::endl;
      // std::cout << "X=" << Xp[0] << ", " << Xp[1] << ", " << Xp[2] << std::endl;
      // std::cout << "Y=" << Yp[0] << ", " << Yp[1] << ", " << Yp[2] << std::endl;
      // std::cout << "Ux=" << Ux[i] << ", Uy=" << Uy[i] << std::endl;

      int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;
      
      // com_x[i] = Xp[0];
      // com_y[i] = Yp[0];
      // std::cout << "(" << t_ms << ", " << com_x[i] << ", " << com_y[i] << "), ___ " << com_x.size() << ", " << com_y.size() << " ___ ";
      // std::cout << t_ms << ": " << Xp[0] << ", " << Xp[1] << ", " << Xp[2] << " - " << u_x[i - k] << std::endl;

      zmp_x[i] = (C * Xp).value();
      zmp_y[i] = (C * Yp).value();
      // std::cout << "(" << t_ms << ", " << zmp_x[i] << ", " << zmp_y[i] << "), ";

      Xp = A * Xp + B * Ux[i];
      Yp = A * Yp + B * Uy[i];
      
      com_x[i] = Xp[0];
      com_y[i] = Yp[0];
      // std::cout << "Ux=" << Ux[i] << ", Uy=" << Uy[i] << std::endl;
      // int j = std::min(k + i, (int) COM_X.size() - 1);
      // com_x[i] = COM_X[j];
      // com_y[i] = COM_Y[j];
    }

    // auto footpose_start = std::chrono::high_resolution_clock::now(); // Start timer

    for (int i = 0; i < L; i++) {
      int j = k + i;
      // int t_ms = (int) i * ZMP_MPC_TIMESTEP_DURATION_MS;

      shi2d2_interfaces::msg::FootPose left_foot_pose; 
      left_foot_pose.leg_id = LEFT_LEG;
      left_foot_pose.x = (0*left_footstep_positions_[j].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      left_foot_pose.y = (0*left_footstep_positions_[j].y - 0*com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      left_foot_pose.z = left_footstep_positions_[j].z + DEFAULT_FOOT_POSITION_Z_M;
      left_foot_pose.rx = 0.0;
      left_foot_pose.ry = 0.0;
      left_foot_pose.rz = 0.0;

      shi2d2_interfaces::msg::FootPose right_foot_pose;
      right_foot_pose.leg_id = RIGHT_LEG;
      right_foot_pose.x = (0*right_footstep_positions_[j].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      right_foot_pose.y = -(0*right_footstep_positions_[j].y - 0*com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      right_foot_pose.z = right_footstep_positions_[j].z + DEFAULT_FOOT_POSITION_Z_M;
      right_foot_pose.rx = 0.0;
      right_foot_pose.ry = 0.0;
      right_foot_pose.rz = 0.0;

      left_foot_poses.push_back(left_foot_pose);
      right_foot_poses.push_back(right_foot_pose);

      // std::cout << "COM X=" << com_x[i] << ", COM Y=" << com_y[i] << std::endl;
      // std::cout << i << " Left foot pose: " << left_foot_poses[i].x << ", " 
      //           << left_foot_poses[i].y << ", " << left_foot_poses[i].z << std::endl;
      // std::cout << i << " Right foot pose: " << right_foot_poses[i].x << ", "
      //           << right_foot_poses[i].y << ", " << right_foot_poses[i].z << std::endl;
      // std::cout << std::endl;
    }

    for (int i = 0; i < L; i++) {
      foot_pose_publisher_->publish(left_foot_poses[i]);
      foot_pose_publisher_->publish(right_foot_poses[i]);
      // rclcpp::sleep_for(std::chrono::milliseconds(5)); // Sleep to avoid flooding the topic
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(50000));

    // auto footpose_end = std::chrono::high_resolution_clock::now(); // End timer
    // auto footpose_duration = std::chrono::duration_cast<std::chrono::milliseconds>(footpose_end - footpose_start).count();

    // std::cout << "Published Foot Poses" << std::endl;
    // std::cout << "Foot pose generation took " << footpose_duration << " ms" << std::endl;

    // std::cout << "COM X=" << com_x[0] << ", COM Y=" << com_y[0] << std::endl;
    // std::cout << k << " Left foot pose: " << left_foot_poses[0].x << ", " 
    //           << left_foot_poses[0].y << ", " << left_foot_poses[0].z << std::endl;
    // std::cout << k << " Right foot pose: " << right_foot_poses[0].x << ", "
    //           << right_foot_poses[0].y << ", " << right_foot_poses[0].z << std::endl;
    // std::cout << std::endl;


    // stuff for plotting solutions

    std::vector<double> zmp_refs_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_refs_y(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> t(ZMP_MPC_NUM_TIMESTEPS);
    for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      t[i] = (k + i) * ZMP_MPC_TIMESTEP_DURATION_MS;
      if (k + i < zmp_refs_x_.size() && k + i < zmp_refs_y_.size()) {
        zmp_refs_x[i] = zmp_refs_x_[k + i];
        zmp_refs_y[i] = zmp_refs_y_[k + i];
      } else {
        zmp_refs_x[i] = 0.0; // pad with zeroes
        zmp_refs_y[i] = 0.0; // pad with zeroes
      }
    }

    if ((int) (tick_count_ * PLANNER_LOOP_PERIOD_MS) % 500 == 0) {
      // plot_zmp_mpc_results(
      //   t, 
      //   com_x, com_y,
      //   zmp_refs_x, zmp_refs_y,
      //   zmp_x, zmp_y,
      //   left_footstep_positions_, right_footstep_positions_
      // );
    }
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

  void timer_callback()
  {
    get_robot_state();
    if (tick_count_ * PLANNER_LOOP_PERIOD_MS >= 1000.0) {
      // Process TF data
      // try
      // {
      //   geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("world", "com_link", tf2::TimePointZero);
      //   geometry_msgs::msg::TransformStamped left_foot_transform_stamped = tf_buffer_->lookupTransform("world", "left_foot_link", tf2::TimePointZero);
      //   geometry_msgs::msg::TransformStamped right_foot_transform_stamped = tf_buffer_->lookupTransform("world", "right_foot_link", tf2::TimePointZero);
      //   body_transform_ = transform_stamped.transform;
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
      // }
      // catch (const tf2::TransformException &ex)
      // {
      //   RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      // }
      // get_robot_state();

      // Example publishing logic (can be customized)
      // std_msgs::msg::Int8 message = std_msgs::msg::Int8();
      // message.data = 0; // Example command
      // publisher_->publish(message);



      // if (((int) (tick_count_ * PLANNER_LOOP_PERIOD_MS)) % 500 == 0 && tick_count_ > 100) {
      //   std::cout << tick_count_ << std::endl;
      //   solve_zmp_mpc();
      // }
      if (tick_count_ + ZMP_MPC_NUM_TIMESTEPS < ZMP_MPC_NUM_FOOTSTEP_PLANNING_TIMESTEPS) {
        // solve_zmp_mpc((int) (tick_count_ - 1000/PLANNER_LOOP_PERIOD_MS));
      }
    }
    
    tick_count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t tick_count_;

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
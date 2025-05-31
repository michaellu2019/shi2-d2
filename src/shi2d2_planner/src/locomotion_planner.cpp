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

    init_zmp_mpc();
    // solve_zmp_mpc();
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
    
    Eigen::Vector3d X(body_state_.x, body_state_.vx, body_state_.ax);
    Eigen::Vector3d Y(body_state_.y, body_state_.vy, body_state_.ay);
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
    std::cout << "Solving ZMP MPC for k=" << k << " took " << zmp_mpc_duration << " ms" << std::endl;
    std::cout << "ZMP REF X=" << ZMP_REFS_X[0] << ", Y=" << ZMP_REFS_Y[0] << std::endl;
    std::cout << "X=" << X[0] << ", " << X[1] << ", " << X[2] << std::endl;
    std::cout << "Y=" << Y[0] << ", " << Y[1] << ", " << Y[2] << std::endl;
    std::cout << "Ux=" << Ux[0] << ", Uy=" << Uy[0] << std::endl;
    // std::cout << std::endl;
    
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

    std::vector<double> COM_X = {-0.0199548, -0.0199505, -0.0199395, -0.0199191, -0.0198874, -0.0198426, -0.0197835, -0.019709, -0.0196185, -0.0195113, -0.0193871, -0.0192458, -0.0190873, -0.0189116, -0.0187189, -0.0185093, -0.0182832, -0.0180407, -0.0177824, -0.0175084, -0.0172192, -0.0169152, -0.0165968, -0.0162643, -0.0159181, -0.0155586, -0.0151861, -0.0148011, -0.0144037, -0.0139944, -0.0135735, -0.0131411, -0.0126976, -0.0122431, -0.0117779, -0.0113022, -0.0108161, -0.0103198, -0.00981336, -0.00929688, -0.00877044, -0.00823408, -0.00768783, -0.0071317, -0.00656566, -0.00598967, -0.00540368, -0.00480759, -0.00420132, -0.00358474, -0.0029577, -0.00232006, -0.00167164, -0.00101225, -0.000341674, 0.000340296, 0.0010339, 0.00173938, 0.002457, 0.00318701, 0.0039297, 0.00468534, 0.00545422, 0.0062366, 0.00703277, 0.007843, 0.00866754, 0.00950665, 0.0103605, 0.0112294, 0.0121134, 0.0130126, 0.0139272, 0.014857, 0.0158021, 0.0167623, 0.0177373, 0.0187267, 0.01973, 0.0207465, 0.0217752, 0.0228151, 0.0238647, 0.0249226, 0.0259872, 0.0270573, 0.0281316, 0.0292092, 0.0302895, 0.0313719, 0.0324559, 0.0335413, 0.0346278, 0.0357153, 0.0368037, 0.0378929, 0.0389829, 0.0400736, 0.0411648, 0.0422565, 0.0433485, 0.0444404, 0.0455318, 0.0466223, 0.0477116, 0.0487991, 0.0498847, 0.0509683, 0.0520499, 0.0531294, 0.0542071, 0.0552831, 0.0563576, 0.0574309, 0.058503, 0.0595743, 0.060645, 0.0617151, 0.0627847, 0.063854, 0.0649228, 0.065991, 0.0670583, 0.0681243, 0.0691887, 0.0702512, 0.0713117, 0.0723701, 0.0734265, 0.0744808, 0.0755334, 0.0765844, 0.077634, 0.0786826, 0.0797302, 0.0807771, 0.0818237, 0.0828699, 0.0839159, 0.0849619, 0.0860076, 0.0870529, 0.0880976, 0.0891413, 0.0901838, 0.0912246, 0.0922637, 0.093301, 0.0943365, 0.0953704, 0.0964027, 0.0974338, 0.0984638, 0.0994929, 0.100521, 0.10155, 0.102578, 0.103606, 0.104634, 0.105662, 0.10669, 0.107718, 0.108746, 0.109773, 0.110799, 0.111824, 0.112847, 0.113869, 0.114889, 0.115907, 0.116924, 0.117941, 0.118956, 0.119971, 0.120985, 0.121999, 0.123013, 0.124028, 0.125042, 0.126057, 0.127072, 0.128087, 0.129102, 0.130116, 0.13113, 0.132142, 0.133152, 0.134162, 0.13517, 0.136176, 0.137181, 0.138186, 0.139189, 0.140192, 0.141195, 0.142197, 0.1432, 0.144203, 0.145206, 0.146209, 0.147213, 0.148217, 0.14922, 0.150223, 0.151225, 0.152226, 0.153225, 0.154223, 0.155219, 0.156214, 0.157208, 0.158201, 0.159193, 0.160184, 0.161175, 0.162165, 0.163156, 0.164147, 0.165137, 0.166129, 0.16712, 0.168111, 0.169101, 0.170091, 0.17108, 0.172068, 0.173053, 0.174037, 0.17502, 0.176001, 0.17698, 0.177958, 0.178935, 0.179911, 0.180886, 0.181861, 0.182835, 0.183809, 0.184783, 0.185757, 0.18673, 0.187703, 0.188675, 0.189646, 0.190616, 0.191583, 0.192549, 0.193512, 0.194473, 0.195432, 0.196389, 0.197344, 0.198297, 0.199248, 0.200199, 0.201148, 0.202096, 0.203043, 0.203989, 0.204934, 0.205878, 0.206821, 0.207762, 0.208701, 0.209638, 0.210572, 0.211502, 0.21243, 0.213354, 0.214275, 0.215193, 0.216108, 0.21702, 0.217929, 0.218836, 0.21974, 0.220642, 0.221542, 0.222439, 0.223334, 0.224227, 0.225117, 0.226004, 0.226887, 0.227766, 0.228641, 0.229511, 0.230377, 0.231238, 0.232094, 0.232945, 0.233792, 0.234634, 0.235473, 0.236307, 0.237138, 0.237965, 0.238789, 0.23961, 0.240427};
    std::vector<double> COM_Y = {4.22747e-06, 2.48749e-06, -2.03949e-06, -1.04498e-05, -2.3664e-05, -4.24517e-05, -6.74525e-05, -9.91947e-05, -0.000138112, -0.000184557, -0.000238813, -0.000301106, -0.000371615, -0.000450474, -0.000537788, -0.000633632, -0.000738057, -0.000851099, -0.000972778, -0.0011031, -0.00124207, -0.00138968, -0.00154592, -0.00171077, -0.00188423, -0.00206627, -0.00225688, -0.00245604, -0.00266374, -0.00287995, -0.00310466, -0.00333784, -0.00357947, -0.00382953, -0.00408797, -0.00435476, -0.00462983, -0.00491311, -0.00520452, -0.00550395, -0.00581126, -0.00612628, -0.00644881, -0.00677858, -0.0071153, -0.00745859, -0.00780801, -0.00816303, -0.008523, -0.00888719, -0.0092547, -0.00962448, -0.00999533, -0.0103658, -0.0107342, -0.0110985, -0.0114566, -0.0118058, -0.0121429, -0.0124644, -0.0127663, -0.0130438, -0.0132914, -0.0135035, -0.0136745, -0.0137993, -0.0138737, -0.0138944, -0.0138587, -0.0137651, -0.0136123, -0.0134001, -0.0131289, -0.0127997, -0.0124145, -0.0119758, -0.0114872, -0.010953, -0.0103788, -0.00977096, -0.00913743, -0.00848741, -0.00783158, -0.00718135, -0.00654811, -0.00594236, -0.00537303, -0.00484752, -0.00437185, -0.00395072, -0.00358771, -0.00328526, -0.00304477, -0.00286664, -0.00275026, -0.00269398, -0.00269517, -0.00275005, -0.00285373, -0.00300004, -0.00318145, -0.00338894, -0.00361195, -0.0038391, -0.00405902, -0.00426116, -0.00443648, -0.00457747, -0.00467796, -0.00473305, -0.00473898, -0.00469309, -0.00459374, -0.00444031, -0.00423315, -0.00397365, -0.00366421, -0.00330831, -0.00291061, -0.002477, -0.00201475, -0.00153262, -0.00104091, -0.000550724, -7.31955e-05, 0.000381391, 0.000804251, 0.00118811, 0.00152706, 0.00181642, 0.0020527, 0.00223346, 0.00235729, 0.0024238, 0.00243359, 0.00238824, 0.00229039, 0.00214374, 0.00195317, 0.00172476, 0.00146601, 0.00118589, 0.000894901, 0.000604359, 0.000325592, 6.90919e-05, -0.000156157, -0.00034268, -0.000484367, -0.000576355, -0.000614932, -0.000597471, -0.000522375, -0.000389046, -0.000197882, 4.97186e-05, 0.000351328, 0.000703437, 0.00110138, 0.00153926, 0.00200979, 0.0025042, 0.0030122, 0.00352268, 0.00402452, 0.00450744, 0.00496268, 0.00538297, 0.00576244, 0.00609643, 0.00638148, 0.00661517, 0.00679616, 0.00692408, 0.00699957, 0.00702429, 0.00700092, 0.00693322, 0.00682612, 0.0066858, 0.0065198, 0.0063372, 0.00614856, 0.0059653, 0.00579883, 0.00565975, 0.00555713, 0.00549856, 0.00549027, 0.00553724, 0.00564329, 0.00581119, 0.00604268, 0.00633847, 0.00669834, 0.00712102, 0.00760426, 0.0081447, 0.00873786, 0.009378, 0.010058, 0.0107694, 0.011502, 0.0122449, 0.0129872, 0.0137189, 0.0144314, 0.0151177, 0.0157722, 0.0163904, 0.0169692, 0.0175065, 0.0180011, 0.018453, 0.0188632, 0.0192336, 0.0195671, 0.0198681, 0.0201416, 0.0203942, 0.0206339, 0.02087, 0.0211137, 0.0213766, 0.0216707, 0.0220069, 0.0223947, 0.0228424, 0.0233564, 0.0239424, 0.0246047, 0.0253465, 0.0261701, 0.0270769, 0.0280672, 0.0291402, 0.0302945, 0.0315272, 0.0328345, 0.0342115, 0.0356517, 0.0371472, 0.0386888, 0.0402662, 0.0418694, 0.0434892, 0.0451178, 0.0467492, 0.0483785, 0.0500024, 0.0516185, 0.0532258, 0.0548242, 0.0564146, 0.0579992, 0.0595809, 0.0611639, 0.0627535, 0.0643562, 0.0659798, 0.0676334, 0.0693278, 0.0710754, 0.0728894, 0.0747829, 0.0767685, 0.0788571, 0.0810585, 0.0833807, 0.0858308, 0.0884148, 0.0911377, 0.0940032, 0.0970144, 0.100173, 0.103481, 0.106937, 0.110541, 0.114291, 0.118183, 0.122212, 0.126372, 0.130655, 0.135053, 0.139556, 0.144156, 0.148847, 0.153624, 0.158481, 0.163416, 0.168427, 0.173511, 0.178668, 0.183897, 0.189198, 0.194571, 0.200016, 0.205534, 0.211125, 0.21679};
    
    // int L = ZMP_MPC_NUM_TIMESTEPS;
    int L = 1;
    
    // change to only use first element of Ux and Uy 
    // only use first com_x and com_y
    // only publish first left_foot_pose and right_foot_pose

    for (int i = 0; i < L; i++) {
      // std::cout << "Solving ZMP MPC for i=" << i << " took " << zmp_mpc_duration << " ms" << std::endl;
      // std::cout << "ZMP REF X=" << ZMP_REFS_X[i] << ", Y=" << ZMP_REFS_Y[i] << std::endl;
      // std::cout << "X=" << Xp[0] << ", " << Xp[1] << ", " << Xp[2] << std::endl;
      // std::cout << "Y=" << Yp[0] << ", " << Yp[1] << ", " << Yp[2] << std::endl;
      // std::cout << "Ux=" << Ux[i] << ", Uy=" << Uy[i] << std::endl;
      // std::cout << std::endl;

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
      left_foot_pose.x = (left_footstep_positions_[j].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      left_foot_pose.y = (0*left_footstep_positions_[j].y - com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      left_foot_pose.z = left_footstep_positions_[j].z + DEFAULT_FOOT_POSITION_Z_M;
      left_foot_pose.rx = 0.0;
      left_foot_pose.ry = 0.0;
      left_foot_pose.rz = 0.0;

      shi2d2_interfaces::msg::FootPose right_foot_pose;
      right_foot_pose.leg_id = RIGHT_LEG;
      right_foot_pose.x = (right_footstep_positions_[j].x - com_x[i]) + DEFAULT_FOOT_POSITION_X_M;
      right_foot_pose.y = -(0*right_footstep_positions_[j].y - com_y[i]) + DEFAULT_FOOT_POSITION_Y_M;
      right_foot_pose.z = right_footstep_positions_[j].z + DEFAULT_FOOT_POSITION_Z_M;
      right_foot_pose.rx = 0.0;
      right_foot_pose.ry = 0.0;
      right_foot_pose.rz = 0.0;

      left_foot_poses.push_back(left_foot_pose);
      right_foot_poses.push_back(right_foot_pose);

      // std::cout << "COM X=" << com_x[i] << ", COM Y=" << com_y[i] << std::endl;
      // std::cout << i << " Left foot pose=" << left_foot_poses[i].x << ", " 
      //           << left_foot_poses[i].y << ", " << left_foot_poses[i].z << std::endl;
      // std::cout << i << " Right foot pose=" << right_foot_poses[i].x << ", "
      //           << right_foot_poses[i].y << ", " << right_foot_poses[i].z << std::endl;
      // std::cout << std::endl;
    }

    for (int i = 0; i < L; i++) {
      foot_pose_publisher_->publish(left_foot_poses[i]);
      foot_pose_publisher_->publish(right_foot_poses[i]);
      // rclcpp::sleep_for(std::chrono::milliseconds(5)); // Sleep to avoid flooding the topic
    }

    // auto footpose_end = std::chrono::high_resolution_clock::now(); // End timer
    // auto footpose_duration = std::chrono::duration_cast<std::chrono::milliseconds>(footpose_end - footpose_start).count();

    // std::cout << "Published Foot Poses" << std::endl;
    // std::cout << "Foot pose generation took " << footpose_duration << " ms" << std::endl;
    std::cout << "COM X=" << com_x[0] << ", COM Y=" << com_y[0] << std::endl;
    std::cout << k << " Left foot pose: " << left_foot_poses[0].x << ", " 
              << left_foot_poses[0].y << ", " << left_foot_poses[0].z << std::endl;
    std::cout << k << " Right foot pose: " << right_foot_poses[0].x << ", "
              << right_foot_poses[0].y << ", " << right_foot_poses[0].z << std::endl;
    std::cout << std::endl;


    // stuff for plotting solutions

    // std::vector<double> zmp_refs_x(ZMP_MPC_NUM_TIMESTEPS);
    // std::vector<double> zmp_refs_y(ZMP_MPC_NUM_TIMESTEPS);
    // std::vector<double> t(ZMP_MPC_NUM_TIMESTEPS);
    // for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
    //   t[i] = (k + i) * ZMP_MPC_TIMESTEP_DURATION_MS;
    //   if (k + i < zmp_refs_x_.size() && k + i < zmp_refs_y_.size()) {
    //     zmp_refs_x[i] = zmp_refs_x_[k + i];
    //     zmp_refs_y[i] = zmp_refs_y_[k + i];
    //   } else {
    //     zmp_refs_x[i] = 0.0; // pad with zeroes
    //     zmp_refs_y[i] = 0.0; // pad with zeroes
    //   }
    // }

    // if ((int) (tick_count_ * PLANNER_LOOP_PERIOD_MS) % 500 == 0) {
    //   plot_zmp_mpc_results(
    //     t, 
    //     com_x, com_y,
    //     zmp_refs_x, zmp_refs_y,
    //     zmp_x, zmp_y,
    //     left_footstep_positions_, right_footstep_positions_
    //   );
    // }
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
        solve_zmp_mpc((int) (tick_count_ - 1000/PLANNER_LOOP_PERIOD_MS));
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
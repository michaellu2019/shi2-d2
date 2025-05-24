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

    // Subscribers
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/shi2d2/imu", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::imu_callback, this, std::placeholders::_1));

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/shi2d2/odometry", PLANNER_LOOP_PERIOD_MS,
      std::bind(&LocomotionPlanner::odometry_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(PLANNER_LOOP_PERIOD_MS * 1ms, std::bind(&LocomotionPlanner::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    zmp_mpc();
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

  static double zmp_mpc_cost_function(unsigned n, const double *u, double *grad, void *data)
  {
    ZMPMPCData *zmp_mpc_data = static_cast<ZMPMPCData *>(data);
    const Eigen::Vector3d &X = zmp_mpc_data->X;
    const Eigen::Matrix<double, 3, 3> &A = zmp_mpc_data->A;
    const Eigen::Matrix<double, 3, 1> &B = zmp_mpc_data->B;
    const Eigen::Matrix<double, 1, 3> &C = zmp_mpc_data->C;
    const std::vector<double> &zmp_refs = zmp_mpc_data->zmp_refs;

    double cost = 0.0;
    int k = 0;
    
    Eigen::Vector3d X_next(X[0], X[1], X[2]);
    for (int i = k; i < k + ZMP_MPC_NUM_TIMESTEPS; i++) {
      // const Eigen::Vector3d X_next = A * X + B * u[i];
      // std::cout << "X (" << X_next[0] << ", " << X_next[1] << ", " << X_next[2] << ") with u:" << u[i];
      X_next = A * X_next + B * u[i];
      double zmp_next = (C * X_next).value();
      double zmp_ref_next = zmp_refs[i + 1];
      // std::cout << " --> (" << X_next[0] << ", " << X_next[1] << ", " << X_next[2] << ")" << std::endl;
      // std::cout << "ZMP VS ZMP REF (" << i << ": " << zmp_next << " vs. " << zmp_ref_next << "), and X (" 
      //           << X_next[0] << ", " << X_next[1] << ", " << X_next[2] << ") with u:" << u[i] << std::endl;

      if (grad) {
        double grad1 = ZMP_MPC_COP_PENALTY * (zmp_next - zmp_ref_next) * (pow(ZMP_MPC_TIMESTEP_PERIOD_SEC, 3)/6 - (ZMP_MPC_COM_HEIGHT_M/g) * ZMP_MPC_TIMESTEP_PERIOD_SEC);
        double grad2 = ZMP_MPC_U_PENALTY * u[i];
        grad[i] = grad1 + grad2;

        // std::cout << "Grad: " << grad[i];
        // grad[i] = ZMP_MPC_COP_PENALTY * (u[i] - zmp_ref_next) + ZMP_MPC_U_PENALTY * (u[i] - zmp_ref_next);
      }

      double iteration_cost1 = 0.5 * ZMP_MPC_COP_PENALTY * pow(zmp_next - zmp_ref_next, 2);
      double iteration_cost2 = 0.5 * ZMP_MPC_U_PENALTY * pow(u[i], 2);
      // double iteration_cost1 = 0.5 * ZMP_MPC_COP_PENALTY * pow(u[i] - zmp_ref_next, 2);
      // double iteration_cost2 = 0.5 * ZMP_MPC_U_PENALTY * pow(u[i] - zmp_ref_next, 2);
      cost += iteration_cost1 + iteration_cost2;

      // std::cout << " Cost: " << iteration_cost1 << " + " << iteration_cost2 << " = " << (iteration_cost1 + iteration_cost2) << std::endl;
    }

    return cost;
  }

  // static double zmp_mpc_constraint(unsigned n, const double *u, double *grad, void *data)
  // {
  //   ZMPMPCData *zmp_mpc_data = static_cast<ZMPMPCData *>(data);
  //   const Eigen::Vector3d &X = zmp_mpc_data->X;
  //   const Eigen::Matrix<double, 3, 3> &A = zmp_mpc_data->A;
  //   const Eigen::Matrix<double, 3, 1> &B = zmp_mpc_data->B;
  //   const Eigen::Matrix<double, 1, 3> &C = zmp_mpc_data->C;

  //   double constraint_value = 0.0;
  //   for (int i = 0; i < n; i++) {
  //     constraint_value += u[i];
  //   }

  //   return constraint_value;
  // }

  void zmp_mpc()
  {
    std::cout << "Testing ZMP MPC" << std::endl;
    Eigen::Vector3d X(body_state_.x, body_state_.vx, body_state_.ax);
    Eigen::Vector3d Y(body_state_.y, body_state_.vy, body_state_.ay);
    std::vector<double> t(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> u_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> u_y(ZMP_MPC_NUM_TIMESTEPS);

    // std::cout << A << std::endl;
    // std::cout << B << std::endl;
    // std::cout << C << std::endl;
    // return;
    

    ZMPMPCData zmp_mpc_x_data{X, A, B, C, {}};
    ZMPMPCData zmp_mpc_y_data{Y, A, B, C, {}};

    // std::cout << "ZMP Reference Values" << std::endl;
    int k = 0;
    double last_zmp_x = 0;
    double last_zmp_y = 0;
    double current_zmp_x = 0;
    double current_zmp_y = 0;
    double zmp_padding = 30;
    for (int i = k; i < zmp_padding; i++) {
      zmp_mpc_x_data.zmp_refs.push_back(0.0);
      zmp_mpc_y_data.zmp_refs.push_back(0.0);
    }

    for (int i = zmp_padding; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
      int step_num = (int) t_ms/half_step_period_ms;
      bool double_support = t_ms % (int) half_step_period_ms < double_support_duration_ms;
      
      double new_zmp_x = body_state_.x + step_num * ZMP_STEP_LENGTH_M;
      double new_zmp_y = body_state_.y + (2 * (step_num % 2) - 1) * ZMP_STEP_WIDTH_M;
      if (t_ms % (int) half_step_period_ms == 0) {
        last_zmp_x = current_zmp_x;
        current_zmp_x = new_zmp_x;
        last_zmp_y = current_zmp_y;
        current_zmp_y = new_zmp_y;
      }

      if (double_support) {
        zmp_mpc_x_data.zmp_refs.push_back(last_zmp_x + ((current_zmp_x - last_zmp_x)/double_support_duration_ms) * (t_ms % (int) half_step_period_ms));
        zmp_mpc_y_data.zmp_refs.push_back(last_zmp_y + ((current_zmp_y - last_zmp_y)/double_support_duration_ms) * (t_ms % (int) half_step_period_ms));
      } else {
        zmp_mpc_x_data.zmp_refs.push_back(current_zmp_x);
        zmp_mpc_y_data.zmp_refs.push_back(current_zmp_y);
      }
      t[i] = t_ms;
    }

    // for (int i = 0; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
    //   int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
    //   std::cout << "(" << t_ms << ", " << zmp_mpc_x_data.zmp_refs[i] << ", " << zmp_mpc_y_data.zmp_refs[i] << "), ";
      // u_x[i] = zmp_mpc_x_data.zmp_refs[i];
      // u_y[i] = zmp_mpc_y_data.zmp_refs[i];
    // }
    // std::cout << std::endl;

    const int N = ZMP_MPC_NUM_TIMESTEPS;
    const double T = ZMP_MPC_TIMESTEP_PERIOD_SEC;
    const double h = ZMP_MPC_COM_HEIGHT_M;
    const double Q = ZMP_MPC_COP_PENALTY;
    const double R = ZMP_MPC_U_PENALTY;
    
    k = 0;

    Eigen::VectorXd ZMP(N);
    Eigen::Vector<double, N> ZMP_REF_X(zmp_mpc_x_data.zmp_refs.data());
    Eigen::Vector<double, N> ZMP_REF_Y(zmp_mpc_y_data.zmp_refs.data());

    Eigen::MatrixXd Px(N, 3);
    Eigen::MatrixXd Pu(N, N);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);

    // std::cout << "Populating Pu and Px... " << std::endl;

    for (int i = 0; i < N; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
      int n = i + 1;
      
      Px(i, 0) = 1;
      Px(i, 1) = n * T;
      Px(i, 2) = pow(n * T, 2)/2 - h/g;
    }

    for (int i = 0; i < N; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
      int n = i + 1;
      for (int j = 0; j <= i; j++) {
        int m = j + 1;
        int coeff = (1 + 3 * (n - m) + 3 * pow(n - m, 2));
        Pu(i, j) = coeff * pow(T, 3)/6 - (h/g) * T;
      }
    }
    
    // std::cout << "\nZMP Ref X: " << std::endl;
    // std::cout << ZMP_REF_X << std::endl;

    // std::cout << "\nPx: " << std::endl;
    // std::cout << Px << std::endl;
    
    // std::cout << "\nPu: " << std::endl;
    // std::cout << Pu << std::endl;

    Eigen::MatrixXd PuPuRQI = (Pu.transpose() * Pu + (R/Q) * I).inverse();
    Eigen::VectorXd PxZ = Px * X - ZMP_REF_X;
    Eigen::VectorXd PyZ = Px * Y - ZMP_REF_Y;

    Eigen::VectorXd Ux = -PuPuRQI * Pu.transpose() * PxZ;
    Eigen::VectorXd Uy = -PuPuRQI * Pu.transpose() * PyZ;

    for (int i = 0; i < N; i++) {
      u_x[i] = Ux[i];
      u_y[i] = Uy[i];
    }

    // std::cout << "\nUx: " << std::endl;
    // std::cout << Ux << std::endl;

    // return;




    // RCLCPP_INFO(this->get_logger(), "Creating Optimization Problems");

    // nlopt::opt opt_x(nlopt::LN_COBYLA, ZMP_MPC_NUM_TIMESTEPS);
    // opt_x.set_min_objective(zmp_mpc_cost_function, &zmp_mpc_x_data);
    // opt_x.set_xtol_rel(1e-4);
    // opt_x.set_maxtime(30 * 60.0);
    // nlopt::opt opt_y(nlopt::LN_COBYLA, ZMP_MPC_NUM_TIMESTEPS);
    // opt_y.set_min_objective(zmp_mpc_cost_function, &zmp_mpc_y_data);
    // opt_y.set_xtol_rel(1e-4);
    // opt_y.set_maxtime(30 * 60.0);
    
    // auto x_start_time = std::chrono::high_resolution_clock::now(); // Start timer
    // double min_x_cost;
    // try {
    //   nlopt::result result = opt_x.optimize(u_x, min_x_cost);
      
    //   auto x_end_time = std::chrono::high_resolution_clock::now(); // End timer
    //   auto x_duration = std::chrono::duration_cast<std::chrono::milliseconds>(x_end_time - x_start_time).count();
    //   RCLCPP_INFO(this->get_logger(), "X Optimization succeeded. Min cost: %.3f", min_x_cost);
    //   RCLCPP_INFO(this->get_logger(), "Solver took %ld ms\n", x_duration);
    //   std::cout << std::endl;
    // } catch (std::exception &e) {
    //   RCLCPP_ERROR(this->get_logger(), "NLOPT failed: %s", e.what());
    // }
    
    // auto y_start_time = std::chrono::high_resolution_clock::now(); // Start timer
    // double min_y_cost;
    // try {
    //   nlopt::result result = opt_y.optimize(u_y, min_y_cost);
      
    //   auto y_end_time = std::chrono::high_resolution_clock::now(); // End timer
    //   auto y_duration = std::chrono::duration_cast<std::chrono::milliseconds>(y_end_time - y_start_time).count();
    //   RCLCPP_INFO(this->get_logger(), "Y Optimization succeeded. Min cost: %.3f", min_y_cost);
    //   RCLCPP_INFO(this->get_logger(), "Solver took %ld ms\n", y_duration);
    //   std::cout << std::endl;
    // } catch (std::exception &e) {
    //   RCLCPP_ERROR(this->get_logger(), "NLOPT failed: %s", e.what());
    // }

    // k = 0;
    // std::cout << "Jerk Values" << std::endl;
    // for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
    //   int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
    //   std::cout << "(" << t_ms << ", " << u_x[i - k] << ", " << u_y[i - k] << "), ";
    // }
    // std::cout << std::endl;

    // std::cout << "ZMP Values" << std::endl;
    std::vector<double> com_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> com_y(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_x(ZMP_MPC_NUM_TIMESTEPS);
    std::vector<double> zmp_y(ZMP_MPC_NUM_TIMESTEPS);
    Eigen::Vector3d Xp(0, 0, 0);
    Eigen::Vector3d Yp(0, 0, 0);
    for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
      int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
      
      com_x[i] = Xp[0];
      com_y[i] = Yp[0];
      // std::cout << "(" << t_ms << ", " << com_x[i] << ", " << com_y[i] << "), ___ " << com_x.size() << ", " << com_y.size() << " ___ ";
      // std::cout << t_ms << ": " << Xp[0] << ", " << Xp[1] << ", " << Xp[2] << " - " << u_x[i - k] << std::endl;

      zmp_x[i] = (C * Xp).value();
      zmp_y[i] = (C * Yp).value();
      // std::cout << "(" << t_ms << ", " << zmp_x[i] << ", " << zmp_y[i] << "), ";

      Xp = A * Xp + B * u_x[i - k];
      Yp = A * Yp + B * u_y[i - k];
      
    }
    std::cout << std::endl;

    // std::cout << "COM Values" << std::endl;
    // for (int i = k; i < ZMP_MPC_NUM_TIMESTEPS; i++) {
    //   int t_ms = (int) i * ZMP_MPC_TIMESTEP_PERIOD_MS;
    //   std::cout << "(" << t_ms << ", " << com_x[i] << ", " << com_y[i] << "), ";
    // }

    plot_results(
      t, com_x, com_y,
      zmp_mpc_x_data.zmp_refs, zmp_mpc_y_data.zmp_refs,
      zmp_x, zmp_y
    );
  }

  void plot_results(const std::vector<double>& time, 
    const std::vector<double>& com_x, const std::vector<double>& com_y,
    const std::vector<double>& zmp_ref_x, const std::vector<double>& zmp_ref_y,
    const std::vector<double>& zmp_x, const std::vector<double>& zmp_y) {
    plt::figure();

    // Plot 1: COM y vs x, ZMP reference y vs x, and ZMP y vs x
    plt::subplot(3, 1, 1); // 3 rows, 1 column, 1st plot
    plt::plot(com_x, com_y, {{"label", "COM y vs x"}});
    plt::plot(zmp_ref_x, zmp_ref_y, {{"label", "ZMP Reference y vs x"}});
    plt::plot(zmp_x, zmp_y, {{"label", "ZMP y vs x"}});
    plt::xlabel("X (meters)");
    plt::ylabel("Y (meters)");
    plt::title("COM and ZMP Trajectories in X-Y Plane");
    plt::legend();
    plt::grid(true);

    // Plot 2: COM x vs t, ZMP reference x vs t, and ZMP x vs t
    plt::subplot(3, 1, 2); // 3 rows, 1 column, 2nd plot
    plt::plot(time, com_x, {{"label", "COM x vs t"}});
    plt::plot(time, zmp_ref_x, {{"label", "ZMP Reference x vs t"}});
    plt::plot(time, zmp_x, {{"label", "ZMP x vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("X (meters)");
    plt::title("COM and ZMP X Trajectories Over Time");
    plt::legend();
    plt::grid(true);

    // // Plot 3: COM y vs t, ZMP reference y vs t, and ZMP y vs t
    plt::subplot(3, 1, 3); // 3 rows, 1 column, 3rd plot
    plt::plot(time, com_y, {{"label", "COM y vs t"}});
    plt::plot(time, zmp_ref_y, {{"label", "ZMP Reference y vs t"}});
    plt::plot(time, zmp_y, {{"label", "ZMP y vs t"}});
    plt::xlabel("Time (seconds)");
    plt::ylabel("Y (meters)");
    plt::title("COM and ZMP Y Trajectories Over Time");
    plt::legend();
    plt::grid(true);

    // Show all plots
    plt::tight_layout();
    plt::show();
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
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Transform body_transform_;
  nav_msgs::msg::Odometry body_odometry_;
  sensor_msgs::msg::Imu body_imu_data_;
  geometry_msgs::msg::Pose body_pose_;

  BodyState body_state_; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionPlanner>());
  rclcpp::shutdown();
  return 0;
}
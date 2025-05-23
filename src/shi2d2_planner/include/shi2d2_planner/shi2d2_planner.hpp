#ifndef SHI2D2_PLANNER_HPP
#define SHI2D2_PLANNER_HPP

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

const double PLANNER_LOOP_PERIOD_MS = 10.0;

const double g = 9.81; // m/s^2

// ZMP linear MPC constants
const int ZMP_MPC_NUM_TIMESTEPS = 300; // number of time steps
const double ZMP_MPC_TIMESTEP_PERIOD_MS = 5.0; // MPC time step period in ms
const double ZMP_MPC_TIMESTEP_PERIOD_SEC = ZMP_MPC_TIMESTEP_PERIOD_MS/1000.0; // MPC time step period in s
const double ZMP_MPC_COM_HEIGHT_M = 0.275;
const double ZMP_MPC_COP_X_BOUNDS_M = 0.75 * 0.140/2;
const double ZMP_MPC_COP_Y_BOUNDS_M = 0.75 * 0.3/2;
const double ZMP_MPC_COP_PENALTY = 1000.0;
const double ZMP_MPC_U_PENALTY = 1000.0 * 1e-6;

double ZMP_STEP_LENGTH_M = 20.0e-3;
double ZMP_STEP_WIDTH_M = 50.0e-3;
double step_period_ms = 400;
double half_step_period_ms = step_period_ms/2;
double single_support_duration_ms = half_step_period_ms - 40;
double double_support_duration_ms = half_step_period_ms - single_support_duration_ms;

Eigen::Matrix<double, 3, 3> A {
  {1, ZMP_MPC_TIMESTEP_PERIOD_SEC, pow(ZMP_MPC_TIMESTEP_PERIOD_SEC, 2)/2},
  {0, 1, ZMP_MPC_TIMESTEP_PERIOD_SEC},
  {0, 0, 1},
};
Eigen::Matrix<double, 3, 1> B {
  pow(ZMP_MPC_TIMESTEP_PERIOD_SEC, 3)/6,
  pow(ZMP_MPC_TIMESTEP_PERIOD_SEC, 2)/2,
  ZMP_MPC_TIMESTEP_PERIOD_SEC,
};
Eigen::Matrix<double, 1, 3> C {
  1, 0, -ZMP_MPC_COM_HEIGHT_M/g,
};

struct ZMPMPCData {
  Eigen::Vector3d X;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  std::vector<double> zmp_refs;
};

struct BodyState {
  // linear and angular position
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  // linear and angular velocity
  double vx;
  double vy;
  double vz;
  double wx;
  double wy;
  double wz;

  // linear and angular acceleration
  double ax;
  double ay;
  double az;
  double alx;
  double aly;
  double alz;
};

#endif
#ifndef SHI2D2_PLANNER_HPP
#define SHI2D2_PLANNER_HPP

#include <string>
#include <vector>

const double PLANNER_LOOP_PERIOD_MS = 10.0;

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
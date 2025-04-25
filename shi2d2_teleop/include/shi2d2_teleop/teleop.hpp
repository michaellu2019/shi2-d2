#ifndef TELEOP_HPP
#define TELEOP_HPP

#include <string>
#include <vector>

const double TELEOP_LOOP_PERIOD_MS = 10.0;

enum WALKING_DIRECTION {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  COUNTERCLOCKWISE = 5,
  CLOCKWISE = 6,
};

#endif
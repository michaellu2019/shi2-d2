#ifndef SHI2D2_CONTROLLER_HPP
#define SHI2D2_CONTROLLER_HPP

#include <string>
#include <vector>

const double CONTROLLER_LOOP_PERIOD_MS = 10;
const double TRAJECTORY_TIME_FROM_START_NS = 0.01e9;
const double TELEOP_COMMAND_TIMEOUT_MS = 200;

enum LEG_ID {
  LEFT_LEG,
  RIGHT_LEG,
};

enum LEG_JOINT_ID {
  UPPER_HIP_BODY_JOINT,
  LOWER_HIP_UPPER_HIP_JOINT,
  UPPER_LEG_LOWER_HIP_JOINT,
  LOWER_LEG_UPPER_LEG_JOINT,
  FOOT_LOWER_LEG_JOINT,
};

enum WALKING_DIRECTION {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  COUNTERCLOCKWISE = 5,
  CLOCKWISE = 6,
};

const int NUM_LEG_JOINTS = 5;
std::vector<std::string> LEFT_LEG_JOINTS = {"left_upper_hip_body_joint", "left_lower_hip_left_upper_hip_joint",
                                            "left_upper_leg_left_lower_hip_joint", "left_lower_leg_left_upper_leg_joint",
                                            "left_foot_left_lower_leg_joint"};
std::vector<std::string> RIGHT_LEG_JOINTS = {"right_upper_hip_body_joint", "right_lower_hip_right_upper_hip_joint",
                                             "right_upper_leg_right_lower_hip_joint", "right_lower_leg_right_upper_leg_joint",
                                             "right_foot_right_lower_leg_joint"};

const int NUM_LEG_LINKS = 5;
std::vector<std::string> LEFT_LEG_LINKS = {"left_upper_hip_link", "left_lower_hip_link",
                                           "left_upper_leg_link", "left_lower_leg_link",
                                           "left_foot_link"};
std::vector<std::string> RIGHT_LEG_LINKS = {"right_upper_hip_link", "right_lower_hip_link",
                                            "right_upper_leg_link", "right_lower_leg_link",
                                            "right_foot_link"};

const double UPPER_HIP_LINK_LENGTH_M = 50e-3;
const double LOWER_HIP_LINK_LENGTH_M = 50e-3;
const double UPPER_LEG_LINK_LENGTH_M = 160e-3;
const double LOWER_LEG_LINK_LENGTH_M = 100e-3;
const double FOOT_LINK_LENGTH_M = 40e-3;

const double BODY_TO_FOOT_DISTANCE_X_M = 0.0;
const double BODY_TO_FOOT_DISTANCE_Y_M = LOWER_HIP_LINK_LENGTH_M;
const double BODY_TO_FOOT_DISTANCE_Z_M = UPPER_HIP_LINK_LENGTH_M + UPPER_LEG_LINK_LENGTH_M + LOWER_LEG_LINK_LENGTH_M + FOOT_LINK_LENGTH_M;

double LEFT_LEG_JOINT_SIGNS[] = {1.0, -1.0, 1.0, 1.0, 1.0};
double LEFT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double RIGHT_LEG_JOINT_SIGNS[] = {-1.0, 1.0, -1.0, 1.0, 1.0};
double RIGHT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};

const double DEFAULT_FOOT_POSITION_X_M = -20.0e-3;
const double DEFAULT_FOOT_POSITION_Y_M = 0.0e-3;
const double DEFAULT_FOOT_POSITION_Z_M = 50.0e-3;
const double DEFAULT_FOOT_ROTATION_RX_RAD = 0.0;
const double DEFAULT_FOOT_ROTATION_RY_RAD = 0.0;
const double DEFAULT_FOOT_ROTATION_RZ_RAD = 0.0;

const double TURN_STEP_ANGLE_RAD = 15 * (M_PI/180.0);
const double FORWARD_STEP_LENGTH_M = 20.0e-3;
const double SIDE_STEP_WIDTH_M = 20.0e-3;
const double STEP_HEIGHT_M = 30.0e-3;
const double STEP_PERIOD_MS = 500;

// speed at which foot travels when sliding along ground for first half of step
// const double STEP_X_SLIDE_SPEED_MM_P_S = STEP_LENGTH_MM/(STEP_PERIOD_MS * 0.5);
// const double STEP_Y_SLIDE_SPEED_MM_P_S = STEP_WIDTH_MM/(STEP_PERIOD_MS * 0.5);

#endif
#ifndef SHI2D2_CONTROLLER_HPP
#define SHI2D2_CONTROLLER_HPP

#include <string>
#include <vector>


const double TRAJECTORY_TIME_FROM_START_NS = 0.01e9;

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

// enum LEFT_LEG_JOINT_ID {
//   LEFT_UPPER_HIP_BODY_JOINT,
//   LEFT_LOWER_HIP_LEFT_UPPER_HIP_JOINT,
//   LEFT_UPPER_LEG_LEFT_LOWER_HIP_JOINT,
//   LEFT_LOWER_LEG_LEFT_UPPER_LEG_JOINT,
//   LEFT_FOOT_LEFT_LOWER_LEG_JOINT,
// };

// enum RIGHT_LEG_JOINT_ID {
//   RIGHT_UPPER_HIP_BODY_JOINT,
//   RIGHT_LOWER_HIP_RIGHT_UPPER_HIP_JOINT,
//   RIGHT_UPPER_LEG_RIGHT_LOWER_HIP_JOINT,
//   RIGHT_LOWER_LEG_RIGHT_UPPER_LEG_JOINT,
//   RIGHT_FOOT_RIGHT_LOWER_LEG_JOINT,
// };

struct LegJointAngles {
  double upper_hip_body_joint_angle;
  double lower_hip_upper_hip_joint_angle;
  double upper_leg_lower_hip_joint_angle;
  double lower_leg_upper_leg_joint_angle;
  double foot_lower_leg_joint_angle;
};

struct Position {
  double x;
  double y;
  double z;
};

struct Rotation {
  double roll;
  double pitch;
  double yaw;
};

struct FootPose {
  Position position;
  Rotation rotation;
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

const double UPPER_HIP_LINK_LENGTH_MM = 50;
const double LOWER_HIP_LINK_LENGTH_MM = 50;
const double UPPER_LEG_LINK_LENGTH_MM = 160;
const double LOWER_LEG_LINK_LENGTH_MM = 100;
const double FOOT_LINK_LENGTH_MM = 40;

const Position BODY_TO_FOOT_DISTANCE_MM = {0.0, LOWER_HIP_LINK_LENGTH_MM, 
                                           UPPER_HIP_LINK_LENGTH_MM + UPPER_LEG_LINK_LENGTH_MM + LOWER_LEG_LINK_LENGTH_MM + FOOT_LINK_LENGTH_MM};

double LEFT_LEG_JOINT_SIGNS[] = {1.0, 1.0, 1.0, 1.0, 1.0};
double LEFT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double RIGHT_LEG_JOINT_SIGNS[] = {1.0, 1.0, 1.0, 1.0, 1.0};
double RIGHT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};

#endif
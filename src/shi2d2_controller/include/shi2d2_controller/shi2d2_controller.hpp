#ifndef SHI2D2_CONTROLLER_HPP
#define SHI2D2_CONTROLLER_HPP

#include <string>
#include <vector>


const double TRAJECTORY_TIME_FROM_START_NS = 0.01e9;

enum LEG_ID {
  LEFT_LEG,
  RIGHT_LEG,
};

enum LEFT_LEG_JOINT_ID {
  LEFT_UPPER_HIP_BDOY_JOINT,
  LEFT_LOWER_HIP_LEFT_UPPER_HIP_JOINT,
  LEFT_UPPER_LEG_LEFT_LOWER_HIP_JOINT,
  LEFT_LOWER_LEG_LEFT_UPPER_LEG_JOINT,
  LEFT_FOOT_LEFT_LOWER_LEG_JOINT,
};

enum RIGHT_LEG_JOINT_ID {
  RIGHT_UPPER_HIP_BDOY_JOINT,
  RIGHT_LOWER_HIP_RIGHT_UPPER_HIP_JOINT,
  RIGHT_UPPER_LEG_RIGHT_LOWER_HIP_JOINT,
  RIGHT_LOWER_LEG_RIGHT_UPPER_LEG_JOINT,
  RIGHT_FOOT_RIGHT_LOWER_LEG_JOINT,
};


const int NUM_LEG_JOINTS = 5;
std::vector<std::string> LEFT_LEG_JOINTS = {"left_upper_hip_body_joint", "left_lower_hip_left_upper_hip_joint",
                                            "left_upper_leg_left_lower_hip_joint", "left_lower_leg_left_upper_leg_joint",
                                            "left_foot_left_lower_leg_joint"};
std::vector<std::string> RIGHT_LEG_JOINTS = {"right_upper_hip_body_joint", "right_lower_hip_right_upper_hip_joint",
                                             "right_upper_leg_right_lower_hip_joint", "right_lower_leg_right_upper_leg_joint",
                                             "right_foot_right_lower_leg_joint"};

struct LegJointAngles {
  double upper_hip_body_joint_angle;
  double lower_hip_upper_hip_joint_angle;
  double upper_leg_lower_hip_joint_angle;
  double lower_leg_upper_leg_joint_angle;
  double foot_lower_leg_joint_angle;
};

struct FootPose {
  double x;
  double y;
  double z;
  double yaw;
};

#endif
#ifndef SHI2D2_CONTROLLER_HPP
#define SHI2D2_CONTROLLER_HPP

#include <string>
#include <vector>

const double CONTROLLER_LOOP_PERIOD_MS = 10.0;
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

double LEFT_LEG_JOINT_SIGNS[] = {1.0, -1.0, 1.0, 1.0, 1.0};
double LEFT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};
double RIGHT_LEG_JOINT_SIGNS[] = {-1.0, 1.0, -1.0, 1.0, 1.0};
double RIGHT_LEG_JOINT_OFFSETS[] = {0.0, 0.0, 0.0, 0.0, 0.0};

Position DEFAULT_FOOT_POSITION = {-20.0, 0.0, 50.0};
Rotation DEFAULT_FOOT_ROTATION = {0.0, 0.0, 0.0};
FootPose DEFAULT_FOOT_POSE = {DEFAULT_FOOT_POSITION, DEFAULT_FOOT_ROTATION};

const double STEP_LENGTH_MM = 30.0;
const double STEP_HEIGHT_MM = 10.0;
const double STEP_PERIOD_MS = 500;

// speed at which foot travels when sliding along ground for first half of step
const double STEP_SLIDE_SPEED_MM_P_S = STEP_LENGTH_MM/(STEP_PERIOD_MS * 0.5);
// speed at which foot travels when lifting off ground for second half of step
// https://en.wikipedia.org/wiki/Perimeter_of_an_ellipse
// const double STEP_LIFT_DISTANCE_TERM_A = 53/3 * STEP_LENGTH_MM/2 + 717/35 * STEP_HEIGHT_MM;
// const double STEP_LIFT_DISTANCE_MM = M_PI * (53/3 * )
// const double STEP_LIFT_SPEED_MM_P_S = (158.65 * 0.5)/(STEP_PERIOD_MS * 0.5);

// Position foot_step_position1 = {0.0, 0.0, DEFAULT_FOOT_Z_MM};
// Rotation foot_step_rotation1 = {0.0, 0.0, 0.0};
// FootPose foot_step_pose1 = {foot_step_position1, foot_step_rotation1};

// Position foot_step_position2 = {-STEP_LENGTH_MM, 0.0, DEFAULT_FOOT_Z_MM};
// Rotation foot_step_rotation2 = {0.0, 0.0, 0.0};
// FootPose foot_step_pose2 = {foot_step_position2, foot_step_rotation2};

// Position foot_step_position3 = {-STEP_LENGTH_MM * (2.0/3.0), 0.0, DEFAULT_FOOT_Z_MM - STEP_HEIGHT_MM};
// Rotation foot_step_rotation3 = {0.0, 0.0, 0.0};
// FootPose foot_step_pose3 = {foot_step_position3, foot_step_rotation3};

// Position foot_step_position4 = {-STEP_LENGTH_MM * (1.0/3.0), 0.0, DEFAULT_FOOT_Z_MM - STEP_HEIGHT_MM};
// Rotation foot_step_rotation4 = {0.0, 0.0, 0.0};
// FootPose foot_step_pose4 = {foot_step_position4, foot_step_rotation4};

// FootPose forward_gait[] = {foot_step_pose1, foot_step_pose2, foot_step_pose3, foot_step_pose4};

#endif
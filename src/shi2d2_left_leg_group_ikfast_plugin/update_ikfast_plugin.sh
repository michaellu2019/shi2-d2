search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=shi2d2.srdf
robot_name_in_srdf=shi2d2
moveit_config_pkg=shi2d2_moveit_config
robot_name=shi2d2
planning_group_name=left_leg_group
ikfast_plugin_pkg=shi2d2_left_leg_group_ikfast_plugin
base_link_name=left_upper_hip_link
eef_link_name=left_foot_link
ikfast_output_path=/home/mlu/Documents/Programming/Robotics/shi2d2/src/shi2d2_left_leg_group_ikfast_plugin/src/shi2d2_left_leg_group_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path

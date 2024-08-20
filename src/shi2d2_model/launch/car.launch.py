# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# gz topic -t "/model/diff_drive/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: -0.1}" && ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}"

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    # pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    # pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    # pkg_project_description = get_package_share_directory('ros_gz_example_description')
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Get the directory where the URDF file is located
    pkg_name = "shi2d2_model"
    pkg_share_path = get_package_share_directory(pkg_name)
    print(f"Package share path: {pkg_share_path}")




    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += pkg_share_path[:-len(f"/{pkg_name}")]
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = pkg_share_path[:-len(f"/{pkg_name}")]

    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] += f'/opt/ros/{os.environ["ROS_DISTRO"]}/lib/'
    else:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = f'/opt/ros/{os.environ["ROS_DISTRO"]}/lib/'


    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_share_path, 'sdf', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

     # Command to launch Ignition Gazebo with the empty SDF world
    world_file = os.path.join(pkg_share_path, 'worlds', 'diff_drive.sdf')
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_share_path, 'rviz', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share_path, 'config', 'diff_drive_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        rviz
    ])

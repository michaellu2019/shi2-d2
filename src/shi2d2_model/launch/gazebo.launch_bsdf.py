# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import xacro
import os
import numpy as np

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')


    # model_pkg_name = "shi2d2_model"
    # model_pkg_share_path = get_package_share_directory(model_pkg_name)
    # robot_xacro_file = os.path.join(model_pkg_share_path, "urdf", "shi2d2_model.urdf.xacro")
    # doc = xacro.parse(open(robot_xacro_file))
    # xacro.process_doc(doc)

    # configure gazebo environment variables to find plugins and resourcs
    # if "GZ_SIM_RESOURCE_PATH" in os.environ:
    #     os.environ["GZ_SIM_RESOURCE_PATH"] += model_pkg_share_path[:-len(f"/{model_pkg_name}")]
    # else:
    #     os.environ["GZ_SIM_RESOURCE_PATH"] = model_pkg_share_path[:-len(f"/{model_pkg_name}")]

    # if "GZ_SIM_SYSTEM_PLUGIN_PATH" in os.environ:
    #     os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] += f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"
    # else:
    #     os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('shi2d2_model'),
                 'urdf', 'shi2d2_model.xacro.urdf']
            ),
        ]
    )

    # robot_description = {'robot_description': ParameterValue(doc.toxml())}
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('shi2d2_model'),
            'config',
            'shi2d2_controller.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # gz_spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=['-topic', 'robot_description',
    #                '-name', 'shi2d2', '-allow_renaming', 'true'],
    # )

    robot_spawn_pose = (0, 0, 0.345*5, 0, 0, np.pi)
    # doc = xacro.parse(open(robot_xacro_file))
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-allow_renaming", "true",
            "-name", "shi2d2", 
            "-topic", "/robot_description",
            "-x", str(robot_spawn_pose[0]),
            "-y", str(robot_spawn_pose[1]),
            "-z", str(robot_spawn_pose[2]),
            "-R", str(robot_spawn_pose[3]),
            "-P", str(robot_spawn_pose[4]),
            "-Y", str(robot_spawn_pose[5]),
            ],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   ],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

# kinda helpful: 
# - https://github.com/gazebosim/ros_gz_project_template/blob/main/ros_gz_example_bringup/launch/diff_drive.launch.py
# - https://gazebosim.org/docs/fortress/ros2_interop/
# - https://gazebosim.org/api/gazebo/6/resources.html
# - https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
# - https://automaticaddison.com/how-to-control-a-robotic-arm-using-ros-2-control-and-gazebo/
# - https://youtu.be/4QKsDf1c4hc
# - https://youtu.be/PM_1Nb9u-N0
# - https://youtu.be/H6YPkXmkdPg
# - https://robotics.stackexchange.com/questions/93267/joint-state-publisher-produces-empty-messages
# - https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

import os
import numpy as np
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():   
    # Get the package share directory and relevant files
    pkg_name = "shi2d2_model"
    pkg_share_path = get_package_share_directory(pkg_name)
    print(f"Package share path: {pkg_share_path}")

    rviz_config_file = os.path.join(pkg_share_path, "rviz", "shi2d2_rviz_settings.rviz")
    bridge_config_file = os.path.join(pkg_share_path, "config", "ros_gz_bridge_config.yaml")
    world_file = os.path.join(pkg_share_path, "worlds", "blank.world")
    robot_xacro_file = os.path.join(pkg_share_path, "urdf", "shi2d2_model.urdf.xacro")

    # configure gazebo environment variables to find plugins and resourcs
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] += pkg_share_path[:-len(f"/{pkg_name}")]
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = pkg_share_path[:-len(f"/{pkg_name}")]

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] += f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"
    else:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"

    # launch the gazebo ignition world
    gz_sim = ExecuteProcess(
        cmd=["ign", "gazebo", world_file],
        output="screen"
    )

    # spawn the urdf model in the world
    robot_spawn_pose = (0, 0, 0.345*5, 0, 0, np.pi)
    doc = xacro.parse(open(robot_xacro_file))
    xacro.process_doc(doc)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", doc.toxml(),
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

    # start robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": doc.toxml()},
            {"rate": 50},
        ]
    )

    # link ros topics and gazebo messages via the bridge file
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_config_file,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen"
    )

    # launch rviz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    # start ros2 control controllers and state broadcasters
    start_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster"],
        output="screen",)
    start_head_controller_cmd = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
            "head_controller"],
        output="screen")
    start_leg_controller_cmd = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
            "leg_controller"],
        output="screen")

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        bridge,
        robot_state_publisher,
        # start_rviz_cmd,
        start_joint_state_broadcaster,
        start_head_controller_cmd,
        start_leg_controller_cmd,
    ])
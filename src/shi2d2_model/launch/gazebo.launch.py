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
# - https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/#Create_a_Launch_File
# - https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html
# - https://gazebosim.org/docs/harmonic/migration_from_ignition/
# - https://robotics.stackexchange.com/questions/114671/odom-and-models-pose-giving-different-values-in-gazebo-sim

# source install/setup.bash && colcon build && ros2 launch shi2d2_model gazebo.launch.py

import os
import numpy as np
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():   
    # Get the package share directory and relevant files
    model_pkg_name = "shi2d2_model"
    model_pkg_share_path = get_package_share_directory(model_pkg_name)
    print(f"Model Package share path: {model_pkg_share_path}")

    rviz_config_file = os.path.join(model_pkg_share_path, "rviz", "shi2d2_rviz_settings.rviz")
    bridge_config_file = os.path.join(model_pkg_share_path, "config", "ros_gz_bridge_config.yaml")
    # controller_config_file = os.path.join(model_pkg_share_path, "config", "shi2d2_controller.yaml")
    world_file = os.path.join(model_pkg_share_path, "worlds", "blank.world")
    robot_xacro_file = os.path.join(model_pkg_share_path, "urdf", "shi2d2_model.urdf.xacro")

    # configure gazebo environment variables to find plugins and resourcs
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        os.environ["GZ_SIM_RESOURCE_PATH"] += model_pkg_share_path[:-len(f"/{model_pkg_name}")]
    else:
        os.environ["GZ_SIM_RESOURCE_PATH"] = model_pkg_share_path[:-len(f"/{model_pkg_name}")]

    if "GZ_SIM_SYSTEM_PLUGIN_PATH" in os.environ:
        os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] += f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"
    else:
        os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"

    # launch the gazebo world
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_file],
        # cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # spawn the shi2d2 robot urdf model in the world
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

    # move the gazebo camera to focus on the shi2d2 robot
    camera_position = {
        "x": -1.12,
        "y": -0.90,
        "z": 0.69,
    }
    camera_orientation = {
        "x": -0.042,
        "y": 0.127,
        "z": 0.314,
        "w": 0.940,
    }
    camera_position_str = "{" + f"position: {camera_position}".replace("'", "")
    camera_orientation_str = f"orientation: {camera_orientation}".replace("'", "") + "}"
    camera_pose_str = f"pose: {camera_position_str} {camera_orientation_str}"
    gazebo_camera = TimerAction(
        period=2.0,  # Delay to ensure Gazebo is initialized
        actions=[
            ExecuteProcess(
                cmd=["gz", "service", "-s", "/gui/move_to/pose", 
                    "--reqtype", "gz.msgs.GUICamera", 
                    "--reptype", "gz.msgs.Boolean", 
                    "--timeout", "2000", 
                    "--req", camera_pose_str],
                output="screen"
            )
        ])

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
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    shi2d2_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_group_controller", "left_leg_group_controller", "right_leg_group_controller"],
    )

    shi2d2_controller = Node(
        package="shi2d2_controller",
        executable="leg_controller",
        arguments=[],
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
        robot_state_publisher,
        rviz,
        joint_state_broadcaster_spawner,
        shi2d2_group_controller_spawner,
        # shi2d2_controller,
        gazebo_camera,
    ])
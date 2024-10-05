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

import os
import numpy as np
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():   
    # Get the package share directory and relevant files
    model_pkg_name = "shi2d2_model"
    model_pkg_share_path = get_package_share_directory(model_pkg_name)
    print(f"Model Package share path: {model_pkg_share_path}")

    rviz_config_file = os.path.join(model_pkg_share_path, "rviz", "shi2d2_rviz_settings.rviz")
    bridge_config_file = os.path.join(model_pkg_share_path, "config", "ros_gz_bridge_config.yaml")
    world_file = os.path.join(model_pkg_share_path, "worlds", "blank.world")
    robot_xacro_file = os.path.join(model_pkg_share_path, "urdf", "shi2d2_model.urdf.xacro")

    moveit_pkg_name = "shi2d2_moveit_config"
    moveit_pkg_share_path = get_package_share_directory(moveit_pkg_name)
    print(f"Model Package share path: {moveit_pkg_share_path}")

    moveit_controllers_file = os.path.join(moveit_pkg_share_path, "config", "moveit_controllers.yaml")
    moveit_ros2_control_file = os.path.join(moveit_pkg_share_path, "config", "ros2_controllers.yaml")
    moveit_srdf_file = os.path.join(moveit_pkg_share_path, "config", "shi2d2.srdf")
    moveit_joint_limits_file_path = os.path.join(moveit_pkg_share_path, "config", "joint_limits.yaml")
    moveit_kinematics_file_path = os.path.join(moveit_pkg_share_path, "config", "kinematics.yaml")
    moveit_pilz_cartesian_limits_file_path = os.path.join(moveit_pkg_share_path, "config", "pilz_cartesian_limits.yaml")
    moveit_initial_positions_file_path = os.path.join(moveit_pkg_share_path, "config", "initial_positions.yaml")
    moveit_rviz_config_file = os.path.join(moveit_pkg_share_path, "config", "moveit.rviz")

    # configure gazebo environment variables to find plugins and resourcs
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] += model_pkg_share_path[:-len(f"/{model_pkg_name}")]
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = model_pkg_share_path[:-len(f"/{model_pkg_name}")]

    if "IGN_GAZEBO_SYSTEM_PLUGIN_PATH" in os.environ:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] += f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"
    else:
        os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/{os.environ['ROS_DISTRO']}/lib/"

    # launch the gazebo ignition world
    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_file],
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

    # start move it
    moveit_config = (
        MoveItConfigsBuilder("shi2d2", package_name="shi2d2_moveit_config")
        .trajectory_execution(file_path=moveit_controllers_file)
        .robot_description_semantic(file_path=moveit_srdf_file)
        .joint_limits(file_path=moveit_joint_limits_file_path)
        .robot_description_kinematics(file_path=moveit_kinematics_file_path)
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=moveit_pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"start_state": {}}
        ],
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
            {"start_state": {"content": moveit_initial_positions_file_path}},
        ],
    )

    # start ros2 controllers and state broadcasters
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_ros2_control_file],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both"
    )
    moveit_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    moveit_head_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_group_controller", "-c", "/controller_manager"],
    )
    moveit_left_leg_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_group_controller", "-c", "/controller_manager"],
    )
    moveit_right_leg_group_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_group_controller", "-c", "/controller_manager"],
    )
    # joint_state_broadcaster = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active",
    #         "joint_state_broadcaster"],
    #     output="screen",)

    # start_head_controller_cmd = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active",
    #         "head_controller"],
    #     output="screen")
    # start_leg_controller_cmd = ExecuteProcess(
    #     cmd=["ros2", "control", "load_controller", "--set-state", "active",
    #         "leg_controller"],
    #     output="screen")

    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster"],
        output="screen",)
    head_group_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "head_group_controller"],
        output="screen",
    )
    left_leg_group_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_leg_group_controller"],
        output="screen",
    )
    right_leg_group_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_leg_group_controller"],
        output="screen",
    )

    return LaunchDescription([
        ign_gazebo,
        spawn_entity,
        bridge,
        robot_state_publisher,
        move_group,
        rviz,
        # joint_state_broadcaster,
        # head_group_controller,
        # left_leg_group_controller,
        # right_leg_group_controller,
        controller_manager,
        moveit_joint_state_broadcaster,
        moveit_head_group_controller,
        moveit_left_leg_group_controller,
        moveit_right_leg_group_controller,
    ])
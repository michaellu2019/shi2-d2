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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xml.etree.ElementTree as ET
import numpy as np
import xacro

def replace_package_xacro(urdf_file, replacement_string):
    # hacky way to replace the package paths in the URDF
    # (becuz I can't figure it out for the life of me how to do it regularly)
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    for element in root.iter():
        for key, value in element.attrib.items():
            if isinstance(value, str) and "model://" in value:
                element.set(key, value.replace("model://shi2d2_model", replacement_string))
            if isinstance(value, str) and "${-pi/2}" in value:
                element.set(key, value.replace("${-pi/2}", str(-np.pi/2)))
            if isinstance(value, str) and "${pi/2}" in value:
                element.set(key, value.replace("${pi/2}", str(np.pi/2)))

    tree.write(urdf_file)

def generate_launch_description():   
    # Get the directory where the URDF file is located
    pkg_name = "shi2d2_model"
    pkg_share_path = get_package_share_directory(pkg_name)
    print(f"Package share path: {pkg_share_path}")

    rviz_config_path = os.path.join(pkg_share_path, 'rviz', 'rviz_settings_2.rviz')

    bridge_config_file = os.path.join(pkg_share_path, 'config', 'ros_gz_bridge_config.yaml')
    controller_config_file = os.path.join(pkg_share_path, 'config', 'shi2d2_controller.yaml')
    world_file = os.path.join(pkg_share_path, 'worlds', 'blank.world')
    # robot_urdf_file = os.path.join(pkg_share_path, 'urdf', 'shi2d2_model.urdf')
    robot_xacro_file = os.path.join(pkg_share_path, 'urdf', 'shi2d2_model.urdf.xacro')
    # robot_xacro_file = os.path.join(pkg_share_path, 'urdf', 'test_control.urdf.xacro')

    # print(f"URDF file: {robot_urdf_file}")
    # replace_package_xacro(world_file, pkg_share_path)
    # replace_package_xacro(robot_urdf_file, pkg_share_path)

    robot_spawn_pose = (0, 0, 0.345*5, 0, 0, 0)

    doc = xacro.parse(open(robot_xacro_file))
    xacro.process_doc(doc)

    # print(doc.toxml())
    # with open(robot_urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += pkg_share_path[:-len(f"/{pkg_name}")]
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = pkg_share_path[:-len(f"/{pkg_name}")]

    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] += f'/opt/ros/{os.environ["ROS_DISTRO"]}/lib/'
    else:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = f'/opt/ros/{os.environ["ROS_DISTRO"]}/lib/'

    # print(f"\n\nTHE WAY: {os.environ['IGN_GAZEBO_RESOURCE_PATH']}\n\n")

     # Command to launch Ignition Gazebo with the empty SDF world
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_share_path, 'sdf', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_ros_gz_sim,
    #         'worlds',
    #         'empty.world'
    #     ])}.items(),
    # )

    # Spawn URDF model in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', doc.toxml(),
            '-allow_renaming', 'true',
            '-name', 'shi2d2', 
            '-topic', '/robot_description', 
            # '-file', robot_urdf_file,
            '-z', str(robot_spawn_pose[2])
            ],
        output='screen'
    )
    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         # '-string', doc.toxml(),
    #         '-allow_renaming', 'true',
    #         '-name', 'diff_drive', 
    #         '-topic', '/robot_description', 
    #         '-file', sdf_file,
    #         '-z', str(robot_spawn_pose[2])
    #         ],
    #     output='screen'
    # )
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'use_sim_time': True},
                    {'rate': 50},]
    )
    
    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': doc.toxml()},
            # {'robot_description': robot_desc},
            {'rate': 50},
        ]
    )
  
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_body_publisher',
        arguments=['0', '0', '0.', '0', '0', '0', '1', 'world', 'body_link'],
        output='screen',
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Start ignition controller
   # Launch joint state broadcaster
    start_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen',)
    start_joint_trajectory_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'head_controller'],
            # 'joint_trajectory_controller'],
        output='screen')
    # load_joint_trajectory_controller_cmd = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #     target_action=spawn_entity,
    #     on_exit=[start_joint_trajectory_controller_cmd],))

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        # bridge_clock,
        bridge,
        # start_joint_state_publisher_cmd,
        # start_joint_state_publisher_gui_node,
        robot_state_publisher,
        # static_transform_publisher_node,
        start_rviz_cmd,
        # joint_trajectory_controller,
        # load_joint_trajectory_controller_cmd,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller_cmd,
    ])
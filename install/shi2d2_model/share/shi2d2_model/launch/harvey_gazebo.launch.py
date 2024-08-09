import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():   
    # Get the directory where the URDF file is located
    pkg_share_path = get_package_share_directory('shi2d2_model')
    print(f"Package share path: {pkg_share_path}")

    urdf_file = os.path.join(pkg_share_path, 'urdf', 'shi2d2_model_gazebo.urdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  pkg_share_path

    # Spawn URDF model in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'shi2d2_model', 
            '-topic', '/robot_description', 
            '-file', urdf_file,
            '-z', '0.4'
            ],
        output='screen'
    )

    # Gz - ROS Bridge for publishing ground truth poses
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',            
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_entity,
        bridge_clock,
    ])
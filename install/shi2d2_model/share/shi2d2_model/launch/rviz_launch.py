import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Define arguments
    pkg_share = FindPackageShare(package="shi2d2_model").find("shi2d2_model")
    model_arg = DeclareLaunchArgument("model", default_value="", description="Path to URDF model file")
    package_arg = DeclareLaunchArgument("package", default_value="shi2d2_model", description="ROS package name")
    
    # Define parameters
    robot_desc_param = LaunchConfiguration("robot_description", 
                                           default=os.path.join(
                                               pkg_share,
                                               "urdf", "shi2d2_model.urdf"))

    rviz_config_arg = DeclareLaunchArgument("rviz_config", 
                                            default_value=os.path.join(
                                                pkg_share,
                                                "rviz", "rviz_settings.rviz"),
                                            description="Path to RViz configuration file")

    # Define nodes
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc_param}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(package_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(LogInfo(msg=TextSubstitution(text="Launching Shi2D2 model with RViz and joint_state_publisher")))

    # ld.add_action(joint_state_publisher_node)
    # ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld

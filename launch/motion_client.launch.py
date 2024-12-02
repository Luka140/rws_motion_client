from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg = "rws_motion_client"
    ld = LaunchDescription()

    # Use IncludeLaunchDescription to include the rws.launch.py file
    rws_launch_file = os.path.join(
        get_package_share_directory(pkg),
        'launch',
        'rws.launch.py'
    )

    rws_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rws_launch_file)        
    )

    motion_client = Node(
        package=pkg,
        executable="motion_client",
        arguments=[],
    )

    acf_node = Node(
        package='ferrobotics_acf',
        executable='acf.py',
        parameters=[
            {'ip': '169.254.200.17',
             'ramp_duration': 0.,
             'frequency': 120,
             'payload': 2.0,
            }]
    )

    grinder_node = Node(
        package="data_gathering",
        executable="grinder_node",
    )

    # Add the actions to the launch description
    ld.add_action(rws_launch)
    ld.add_action(motion_client)
    ld.add_action(acf_node)
    ld.add_action(grinder_node)

    return ld

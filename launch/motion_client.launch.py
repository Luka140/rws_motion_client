from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg = "rws_motion_client"
    ld = LaunchDescription()

    # Declare launch arguments for configurable parameters
    ld.add_action(DeclareLaunchArgument('symbol_home_flag', default_value='waiting_at_home'))
    ld.add_action(DeclareLaunchArgument('symbol_grind0_flag', default_value='waiting_at_grind0'))
    ld.add_action(DeclareLaunchArgument('symbol_grind_done_flag', default_value='finished_grind_pass'))
    ld.add_action(DeclareLaunchArgument('symbol_run_status_flag', default_value='run_status'))
    ld.add_action(DeclareLaunchArgument('symbol_nr_passes', default_value='num_pass'))
    ld.add_action(DeclareLaunchArgument('symbol_tcp_speed', default_value='tcp_feedrate'))
    ld.add_action(DeclareLaunchArgument('bool_timer_period', default_value='100'))
    ld.add_action(DeclareLaunchArgument('max_tcp_speed', default_value='35.0'))
    ld.add_action(DeclareLaunchArgument('grinder_spinup_duration', default_value='4'))

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
        parameters=[
            {
                'symbol_home_flag': LaunchConfiguration('symbol_home_flag'),
                'symbol_grind0_flag': LaunchConfiguration('symbol_grind0_flag'),
                'symbol_grind_done_flag': LaunchConfiguration('symbol_grind_done_flag'),
                'symbol_run_status_flag': LaunchConfiguration('symbol_run_status_flag'),
                'symbol_nr_passes': LaunchConfiguration('symbol_nr_passes'),
                'symbol_tcp_speed': LaunchConfiguration('symbol_tcp_speed'),
                'bool_timer_period': LaunchConfiguration('bool_timer_period'),
                'max_tcp_speed': LaunchConfiguration('max_tcp_speed'),
                'grinder_spinup_duration': LaunchConfiguration('grinder_spinup_duration'),
            }
        ]
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

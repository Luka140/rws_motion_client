from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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



def generate_launch_description():
    pkg = "abb_motion_client_cpp"

    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    robot_nickname = LaunchConfiguration("robot_nickname")
    polling_rate = LaunchConfiguration("polling_rate")
    no_connection_timeout = LaunchConfiguration("no_connection_timeout")

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    rws_ip = LaunchConfiguration("robot_ip")
    rws_port = LaunchConfiguration("robot_port")
    configure_via_rws = LaunchConfiguration("configure_via_rws")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")


    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.125.1",
            description="IP address to the robot controller's RWS server",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="80",
            description="Port number of the robot controller's RWS server",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_nickname",
            default_value="",
            description="Arbitrary user nickname/identifier for the robot controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "no_connection_timeout",
            default_value="false",
            description="Specifies whether the node is allowed to wait indefinitely \
            for the robot controller during initialization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "polling_rate",
            default_value="10.0",
            description="The frequency [Hz] at which the controller state is collected.",
        )
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "configure_via_rws",
            default_value="true",
            description="If false, the robot description will be generate from joint information \
            in the ros2_control xacro. Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="rws_client/",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )


    rws_node = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"robot_port": robot_port},
            {"robot_nickname": robot_nickname},
            {"polling_rate": polling_rate},
            {"no_connection_timeout": no_connection_timeout},
        ],
    )

    description_package = "abb_irb1200_support" 
    description_file = "irb1200_5_90.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "rws_ip:=",
            rws_ip,
            " ",
            "rws_port:=",
            rws_port,
            " ",
            "configure_via_rws:=",
            configure_via_rws,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=prefix,
        parameters=[robot_description,
            {
                'frame_prefix':prefix
            }],
    )


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg), "config", "rviz_config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    motion_client = Node(
        package=pkg,
        executable="rws_motion_client",
        arguments=[],
    )

    return LaunchDescription(declared_arguments + [rws_node, motion_client, robot_state_publisher_node, rviz_node])

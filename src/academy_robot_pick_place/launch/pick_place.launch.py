import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("academy_robot_gazebo_ignition")

    spawn_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "spawn_world.launch.py")
        ),
        launch_arguments={
            "spawn_screw": "true",
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "robot": "academy_robot",
            "robot_model": "academy_robot_plus",
            "has_arm": "true",
        }.items(),
    )

    object_detector_node = Node(
        package="academy_robot_pick_place",
        executable="object_detector_node",
        name="object_detector_node",
        output="screen",
        emulate_tty=True,
    )

    pick_place_server = Node(
        package="academy_robot_pick_place",
        executable="pick_place_server",
        name="pick_place_server",
        output="screen",
        emulate_tty=True,
    )

    pick_place_client_node = Node(
        package="academy_robot_pick_place",
        executable="pick_place_client_node",
        name="pick_place_client_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"result_timeout_sec": LaunchConfiguration("result_timeout_sec")},
        ],
        condition=IfCondition(LaunchConfiguration("launch_client")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_client",
            default_value="true",
            description="Start client automatically.",
        ),
        DeclareLaunchArgument(
            "result_timeout_sec",
            default_value="900.0",
            description="Client timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "robot_delay_sec",
            default_value="12.0",
            description="Delay before spawning robot after world launch.",
        ),
        DeclareLaunchArgument(
            "detector_delay_sec",
            default_value="45.0",
            description="Delay before starting object_detector_node.",
        ),
        DeclareLaunchArgument(
            "server_delay_sec",
            default_value="70.0",
            description="Delay before starting pick_place_server.",
        ),
        DeclareLaunchArgument(
            "client_delay_sec",
            default_value="95.0",
            description="Delay before starting pick_place_client_node.",
        ),

        spawn_world,

        TimerAction(
            period=LaunchConfiguration("robot_delay_sec"),
            actions=[spawn_robot],
        ),

        TimerAction(
            period=LaunchConfiguration("detector_delay_sec"),
            actions=[object_detector_node],
        ),

        TimerAction(
            period=LaunchConfiguration("server_delay_sec"),
            actions=[pick_place_server],
        ),

        TimerAction(
            period=LaunchConfiguration("client_delay_sec"),
            actions=[pick_place_client_node],
        ),
    ])

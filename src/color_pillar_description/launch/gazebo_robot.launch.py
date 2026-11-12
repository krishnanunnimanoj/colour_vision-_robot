import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    world_file = PathJoinSubstitution([
        FindPackageShare("color_pillar_description"),
        "worlds",
        "color_pillar_world.world"
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("color_pillar_description"),
        "urdf",
        "color_robot.urdf.xacro"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "world": world_file
        }.items()
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "color_robot",
            "-x", "0",
            "-y", "0",
            "-z", "0"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        )
    ])

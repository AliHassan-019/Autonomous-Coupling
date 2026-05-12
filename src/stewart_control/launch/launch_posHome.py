#!/home/rem/rtimulib-env/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="stewart_control",
                executable="posHome_node",
                name="posHome_node",
                output="screen",
            )
        ]
    )

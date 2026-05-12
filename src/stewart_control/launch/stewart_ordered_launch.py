#!/home/rem/rtimulib-env/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package="stewart_control",
        executable="imu_node",
        name="imu_node",
        output="screen",
    )


    aruco_node = Node(
        package="stewart_control",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
    )

    fusion_node = Node(
        package="stewart_control",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
    )

    stewart_node = Node(
        package="stewart_control",
        executable="stewart_node",
        name="stewart_node",
        output="screen",
    )

    return LaunchDescription(
        [
            imu_node,
            aruco_node,
            fusion_node,
            stewart_node,
        ]
    )

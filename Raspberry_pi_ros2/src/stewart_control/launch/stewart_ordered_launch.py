#!/home/rem/rtimulib-env/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Lancer imu_node immédiatement
    imu_node = Node(
        package="stewart_control",
        executable="imu_node",
        name="imu_node",
    )

    # Lancer aruco_node immédiatement
    aruco_node = Node(
        package="stewart_control",
        executable="aruco_node",
        name="aruco_node",
    )

    # Lancer fusion_node immédiatement
    fusion_node = Node(
        package="stewart_control",
        executable="fusion_node",
        name="fusion_node",
    )

    return LaunchDescription(
        [
            imu_node,
            aruco_node,
            fusion_node,
        ]
    )

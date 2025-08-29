#!/home/rem/rtimulib-env/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # ----- Interface désactivée pour l'instant -----
    # interface_node = Node(
    #     package='stewart_control',
    #     executable='interface_node',
    #     name='interface_node',
    # )

    # Lancer imu_node immédiatement
    imu_node = Node(
        package='stewart_control',
        executable='imu_node',
        name='imu_node',
    )

    # Lancer aruco_node immédiatement
    aruco_node = Node(
        package='stewart_control',
        executable='aruco_node',
        name='aruco_node',
    )

    # Lancer stewart_node après 5 secondes
    stewart_node = TimerAction(
        period=5.0,  # délai en secondes
        actions=[
            Node(
                package='stewart_control',
                executable='stewart_node',
                name='stewart_node'
            )
        ]
    )

    return LaunchDescription([
        # interface_node,
        imu_node,
        aruco_node,
        stewart_node
    ])

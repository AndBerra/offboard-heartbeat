from launch import LaunchDescription
from launch.actions import (
    TimerAction,
)
from launch_ros.actions import Node


def generate_launch_description():

    # * --- node ---
    # offboard
    offboard_node = Node(
        package="offboard_heartbeat",
        executable="offboard_heartbeat_node",
        name="offboard_heartbeat_node",
        output="screen",
    )

    return LaunchDescription(
        [
            TimerAction(period=10.0, actions=[offboard_node]),
        ]
    )

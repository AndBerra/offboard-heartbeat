from launch import LaunchDescription
from launch.actions import (
    TimerAction,
)
from launch_ros.actions import Node


def generate_launch_description():

    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from launch.actions import DeclareLaunchArgument

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for topics"
    )

    offboard_node = Node(
        package="offboard_heartbeat",
        executable="offboard_heartbeat_node",
        name="offboard_heartbeat_node",
        output="screen",
        arguments=["--namespace", LaunchConfiguration("namespace")],
    )

    return LaunchDescription(
        [
            namespace_arg,
            TimerAction(period=10.0, actions=[offboard_node]),
        ]
    )

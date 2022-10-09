import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    pkg_name = "garden_control"

    config = os.path.join(
        get_package_share_directory(pkg_name),
        "config", "params.yaml"
    )

    zigbee_bridge = Node(
        package=pkg_name,
        executable="zigbee_bridge",
        name="zigbee_bridge",
        parameters=[config]
    )
    ld.add_action(zigbee_bridge)

    return ld

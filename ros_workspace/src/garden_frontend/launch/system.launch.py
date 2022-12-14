import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    pkg_name = "garden_frontend"

    config = os.path.join(
        get_package_share_directory(pkg_name),
        "config", "params.yaml"
    )

    frontend_rt = Node(
        package=pkg_name,
        executable="frontend",
        name="frontend_rt",
        parameters=[config]
    )
    ld.add_action(frontend_rt)

    frontend_db = Node(
        package=pkg_name,
        executable="frontend",
        name="frontend_db",
        parameters=[config]
    )
    ld.add_action(frontend_db)

    return ld

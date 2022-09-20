import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    pkg_name = "tank_control"
    water_tank_ns = "water_tank"
    nutri_tank_ns = "nutri_tank"

    config = os.path.join(
        get_package_share_directory(pkg_name),
        "config", "params.yaml"
    )

    uc_bridge = Node(
        package=pkg_name,
        executable="uc_bridge",
        name="uc_bridge",
        parameters=[config]
    )
    ld.add_action(uc_bridge)

    water_tank_level_observer = Node(
        package=pkg_name,
        namespace=water_tank_ns,
        executable="level_observer",
        name="level_observer",
        parameters=[config],
        remappings=[
            ("/level_raw", f"/{water_tank_ns}/level_raw")
        ]
    )
    ld.add_action(water_tank_level_observer)

    nutri_tank_level_observer = Node(
        package=pkg_name,
        namespace=nutri_tank_ns,
        executable="level_observer",
        name="level_observer",
        parameters=[config],
        remappings=[
            ("/level_raw", f"/{nutri_tank_ns}/level_raw")
        ]
    )
    ld.add_action(nutri_tank_level_observer)

    water_pump_driver = Node(
        package=pkg_name,
        namespace=water_tank_ns,
        executable="pump_driver",
        name="water_pump_driver",
        parameters=[config]
    )
    ld.add_action(water_pump_driver)

    nutri_pump_driver = Node(
        package=pkg_name,
        namespace=nutri_tank_ns,
        executable="pump_driver",
        name="nutri_pump_driver",
        parameters=[config]
    )
    ld.add_action(nutri_pump_driver)

    return ld

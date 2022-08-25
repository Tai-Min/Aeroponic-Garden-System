import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("tank_control"),
        "config", "params.yaml"
    )

    uc_bridge = Node(
        package="tank_control",
        executable="uc_bridge",
        name="uc_bridge",
        parameters=[config]
    )
    ld.add_action(uc_bridge)

    water_tank_level_observer = Node(
        package="tank_control",
        namespace="water_tank",
        executable="level_observer",
        name="level_observer",
        parameters=[config],
        remappings=[
            ("/level_raw", "/water_tank/level_raw")
        ]
    )
    ld.add_action(water_tank_level_observer)

    nutri_tank_level_observer = Node(
        package="tank_control",
        namespace="nutri_tank",
        executable="level_observer",
        name="level_observer",
        parameters=[config],
        remappings=[
            ("/level_raw", "/nutri_tank/level_raw")
        ]
    )
    ld.add_action(nutri_tank_level_observer)

    #water_pump_driver = Node(
    #    package="tank_control",
    #    namespace="water_tank",
    #    executable="pump_driver",
    #    name="pump_driver",
    #    parameters=[config]
    #)
    #ld.add_action(water_pump_driver)

    return ld

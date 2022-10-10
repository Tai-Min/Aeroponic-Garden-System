import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    launch_tank_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tank_control'),
                'system.launch.py')
        )
    )
    ld.add_action(launch_tank_control)

    launch_garden_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('garden_control'),
                'system.launch.py')
        )
    )
    ld.add_action(launch_garden_control)

    launch_frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('garden_frontend'),
                'system.launch.py')
        )
    )
    ld.add_action(launch_frontend)

    return ld

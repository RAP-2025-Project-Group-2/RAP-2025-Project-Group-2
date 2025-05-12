from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Path to the launch file from icclab_summit_xl package
    summit_xl_simulation_launch_file = os.path.join(
        FindPackageShare("icclab_summit_xl").find("icclab_summit_xl"),
        "launch",
        "summit_xl_simulation_ign.launch.py",
    )

    # Path to the navigation launch file
    summit_xl_nav2_launch_file = os.path.join(
        FindPackageShare("icclab_summit_xl").find("icclab_summit_xl"),
        "launch",
        "summit_xl_nav2.launch.py",
    )

    # Declare the launch arguments for the included launch file (if any)
    # For example, if summit_xl_simulation_ign.launch.py accepts 'world_name':
    # summit_xl_launch_args = {
    #     'world_name': 'my_custom_world'
    # }.items()

    return LaunchDescription(
        [
            # Include the Summit XL simulation launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
                # launch_arguments=summit_xl_launch_args  # Uncomment if you have launch arguments
            ),
            # Include the Summit XL navigation launch file with SLAM enabled
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
                launch_arguments={"slam": "True"}.items(),
            ),
        ]
    )

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # --- Declare and configure launch arguments ---
    declare_slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="False",  # Default is 'False' (string)
        description="Enable SLAM. If True, SLAM is used. If False, a pre-existing map is used.",
    )
    slam = LaunchConfiguration("slam") == "True"

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

    # Path to the MoveIt launch file
    summit_xl_move_it_launch_file = os.path.join(
        FindPackageShare("icclab_summit_xl").find("icclab_summit_xl"),
        "launch",
        "summit_xl_move_it.launch.py",
    )

    # Path to the explore_lite launch file
    explore_lite_launch_file = os.path.join(
        FindPackageShare("explore_lite").find("explore_lite"),
        "launch",
        "explore.launch.py",
    )

    launch_description = []

    # Include the Summit XL simulation launch file
    launch_description.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
            launch_arguments={
                "world": "/home/bonsai/Development/rap/Gruppe2/world/small_house.world"
            }.items(),
        )
    )

    if slam:
        # Include the Summit XL navigation launch file with SLAM enabled
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
                launch_arguments={"slam": "True"}.items(),
            )
        )

        # Include the explore_lite launch file
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(explore_lite_launch_file),
                launch_arguments={
                    "namespace": "/summit",
                    "use_sim_time": "true",
                }.items(),
            )
        )

        # Command to stop exploration immediately after starting
        launch_description.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/summit/explore/resume",
                    "std_msgs/msg/Bool",
                    "{data: false}",
                ],
            )
        )
    else:
        # Include the Summit XL navigation launch file with SLAM enabled
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
                launch_arguments={
                    "map": "/home/ros/rap/Gruppe2/maps/default.yaml"
                }.items(),
            )
        )

    # Include the Summit XL MoveIt launch file
    # launch_description.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(summit_xl_move_it_launch_file),
    #     )
    # )

    return LaunchDescription(launch_description)

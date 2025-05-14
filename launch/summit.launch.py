from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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

    # Include the Summit XL MoveIt launch file
    # launch_description.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(summit_xl_move_it_launch_file),
    #     )
    # )

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

    return LaunchDescription(launch_description)

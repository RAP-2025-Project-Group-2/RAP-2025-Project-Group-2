from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # --- Declare and configure launch arguments ---
    declare_slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="False",  # Default is 'False' (string)
        description="Enable SLAM. If True, SLAM is used. If False, a pre-existing map is used.",
    )

    # Path to the world file within the rosa_summit package
    world_file_path = PathJoinSubstitution(
        [FindPackageShare("rosa_summit"), "world", "small_house.world"]
    )

    # Path to the map file within the rosa_summit package
    map_file_path = PathJoinSubstitution(
        [FindPackageShare("rosa_summit"), "maps", "default.yaml"]
    )

    # Path to the launch file from icclab_summit_xl package
    summit_xl_simulation_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("icclab_summit_xl"),
            "launch",
            "summit_xl_simulation_ign.launch.py",
        ]
    )

    # Path to the navigation launch file
    summit_xl_nav2_launch_file = PathJoinSubstitution(
        [FindPackageShare("icclab_summit_xl"), "launch", "summit_xl_nav2.launch.py"]
    )

    # Path to the explore_lite launch file
    explore_lite_launch_file = PathJoinSubstitution(
        [FindPackageShare("explore_lite"), "launch", "explore.launch.py"]
    )

    actions_if_slam = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
            launch_arguments={"world": world_file_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
            launch_arguments={"slam": "True"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_lite_launch_file),
            launch_arguments={
                "namespace": "/summit",
                "use_sim_time": "True",
            }.items(),
        ),
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
        ),
    ]

    actions_if_no_slam = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
            launch_arguments={"world": world_file_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
            launch_arguments={"map": map_file_path}.items(),
        ),
    ]

    def evaluate_slam_and_select_actions(context, *args, **kwargs):
        slam_value = context.launch_configurations["slam"]
        if slam_value.lower() == "true":
            return actions_if_slam
        else:
            return actions_if_no_slam

    return LaunchDescription(
        [declare_slam_arg, OpaqueFunction(function=evaluate_slam_and_select_actions)]
    )

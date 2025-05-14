from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts
import os
from ament_index_python.packages import get_package_share_directory
import pathlib
import subprocess
from typing import Tuple


def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)


# Helper function to get the maps directory
def _get_maps_dir() -> str:
    """Gets the absolute path to the 'maps' directory in the rosa_summit package, creates it if it doesn't exist."""

    maps_dir = "/home/ros/rap/Gruppe2/maps"
    pathlib.Path(maps_dir).mkdir(parents=True, exist_ok=True)
    return maps_dir


@tool
def send_vel(velocity: float) -> str:
    """
    Sets the forward velocity of the robot.

    :param velocity: the velocity at which the robot should move
    """
    print("Setting velocity to %s" % velocity)
    cmd = (
        "ros2 topic pub -1 /summit/cmd_vel geometry_msgs/msg/Twist '{linear: {x: "
        + str(velocity)
        + ", y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}'"
    )
    success, output = execute_ros_command(cmd)
    if success:
        return "Velocity set to %s" % velocity
    else:
        return "Failed setting velocity"


@tool
def stop() -> str:
    """
    Stops or halts the robot by setting its velocity to zero

    """
    cmd = "ros2 topic pub -1 /summit/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}'"
    success, output = execute_ros_command(cmd)
    if success:
        return "Robot was stopped"
    else:
        return "Failed stopping robot"


@tool
def toggle_auto_exploration(resume_exploration: bool) -> str:
    """
    Starts or stops the autonomous exploration.

    :param resume_exploration: True to start/resume exploration, False to stop/pause exploration.
    """
    cmd = f"ros2 topic pub --once /summit/explore/resume std_msgs/msg/Bool '{{data: {str(resume_exploration).lower()}}}'"
    success, output = execute_ros_command(cmd)
    if success:
        return f"Exploration {'resumed' if resume_exploration else 'paused'}"
    else:
        return f"Failed to {'resume' if resume_exploration else 'pause'} exploration"


@tool
def get_map() -> str:
    """
    Retrieves the occupancy grid map from the /summit/map topic.
    """
    cmd = "ros2 topic echo --once /summit/map nav_msgs/msg/OccupancyGrid"
    success, output = execute_ros_command(cmd)
    if success:
        return f"Map data retrieved: {output}"
    else:
        return "Failed to retrieve map data"


@tool
def navigate_to_pose(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot to an absolute position on the map.

    :param x: The x coordinate of the target position.
    :param y: The y coordinate of the target position.
    :param z_orientation: The z component of the target orientation (quaternion).
    :param w_orientation: The w component of the target orientation (quaternion).
    """
    cmd = (
        f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{{ "
        f"pose: {{ "
        f'header: {{ frame_id: "map" }}, '
        f"pose: {{ "
        f"position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {z_orientation}, w: {w_orientation}}} "
        f"}} }} }}"
    )
    success, output = execute_ros_command(cmd)
    if success:
        # The output of send_goal can be verbose, let's return a simpler message.
        # We might want to parse the output to see if the goal was accepted.
        if "Goal accepted" in output:
            return f"Navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}. Waiting for result..."
        else:
            return f"Navigation goal to x: {x}, y: {y} might have been sent. Output: {output}"
    else:
        return f"Failed to send navigation goal. Error: {output}"


@tool
def navigate_relative(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot relative to its current position.

    :param x: The x coordinate of the target position relative to the robot.
    :param y: The y coordinate of the target position relative to the robot.
    :param z_orientation: The z component of the target orientation (quaternion) relative to the robot.
    :param w_orientation: The w component of the target orientation (quaternion) relative to the robot.
    """
    cmd = (
        f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{{ "
        f"pose: {{ "
        f'header: {{ frame_id: "summit/base_link" }}, '
        f"pose: {{ "
        f"position: {{x: {x}, y: {y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {z_orientation}, w: {w_orientation}}} "
        f"}} }} }}'"
    )
    success, output = execute_ros_command(cmd)
    if success:
        if "Goal accepted" in output:
            return f"Relative navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}. Waiting for result..."
        else:
            return f"Relative navigation goal to x: {x}, y: {y} might have been sent. Output: {output}"
    else:
        return f"Failed to send relative navigation goal. Error: {output}"


@tool
def save_map(map_name: str) -> str:
    """
    Saves the current map from the /summit/map topic to .yaml and .pgm files
    in the 'maps' directory of the 'rosa_summit' package.

    :param map_name: The name for the map (e.g., 'my_lab_map'). Do not include file extensions.
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(
        maps_dir
    ):  # Should be created by _get_maps_dir, but double check
        return f"Error: Maps directory {maps_dir} could not be accessed or created."

    filepath_prefix = os.path.join(maps_dir, map_name)

    # Added --ros-args -r map:=/summit/map to specify the topic
    cmd = f"ros2 run nav2_map_server map_saver_cli -f '{filepath_prefix}' --ros-args -r map:=/summit/map"
    success, output = execute_ros_command(cmd)
    if success:
        if "Map saved to" in output:
            return f"Map successfully saved as {map_name} in {maps_dir}"
        else:
            return f"Map saving process initiated for {map_name} in {maps_dir}. Output: {output}"
    else:
        return f"Failed to save map {map_name} in {maps_dir}. Error: {output}"


@tool
def list_saved_maps() -> str:
    """
    Lists all saved maps in the 'maps' directory of the 'rosa_summit' package.
    Returns a list of map names (without .yaml extension).
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(maps_dir):
        return "Maps directory not found or is not a directory."

    try:
        files = os.listdir(maps_dir)
        map_files = [
            f[:-5]
            for f in files
            if f.endswith(".yaml") and os.path.isfile(os.path.join(maps_dir, f))
        ]
        if not map_files:
            return "No saved maps found in the maps directory."
        return f"Available maps: {', '.join(map_files)}"
    except Exception as e:
        return f"Error listing maps: {e}"


def main():
    print("Hi from rosa_summit.")

    # Get the current username
    user_name = os.getenv("USER")

    try:
        if user_name == "ros":
            print("Using remote Ollama instance")
            llm = ChatOllama(
                model="hhao/qwen2.5-coder-tools:latest",  # "gemma3:12b",  # or your preferred model
                temperature=0,
                num_ctx=32192,  # adjust based on your model's context window
                base_url="http://160.85.252.236:11434",
            )
        else:
            print("Using Anthropic API with Claude Sonnet 3.5")
            # Read API key from file
            try:
                with open("/home/ros/rap/Gruppe2/api-key.txt", "r") as f:
                    # Skip the comment line if it exists
                    api_key = f.read().strip().split("\n")[-1]
            except Exception as e:
                print(f"Error reading API key: {e}")
                return

            llm = ChatAnthropic(
                model="claude-3-5-sonnet-20240620",
                temperature=0,
                anthropic_api_key=api_key,
                max_tokens=4096,
            )
    except Exception as e:
        print(f"Error initializing LLM: {e}")
        return

    prompt = RobotSystemPrompts()
    prompt.critical_instructions = "Before using a tool that performs an action, respond by saying which tool will be invoked and with which parameters. Always ask for confirmation before calling the tool."

    # Pass the LLM to ROSA with both tools available
    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            send_vel,
            stop,
            toggle_auto_exploration,
            get_map,
            navigate_to_pose,
            navigate_relative,
            save_map,
            load_map,
            list_saved_maps,
        ],
        prompts=prompt,
    )

    print("Type 'exit' or 'quit' to end the program")

    try:
        while True:
            msg = input("Enter your request: ")
            if msg.lower() in ["exit", "quit"]:
                break

            try:
                print("Request sent")
                res = agent.invoke(msg)
                if isinstance(res, (list, tuple)):
                    print(res[0])
                else:
                    print(res)
            except Exception as e:
                print(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    agent.shutdown()
    print("Bye from rosa_summit.")


if __name__ == "__main__":
    main()

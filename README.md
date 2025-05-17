# ROSA Summit

This package provides a ROS2 interface for controlling a simulated Summit XL robot using a Large Language Model (LLM) through the ROSA framework.

## Setup

1.  **Clone the repository:**
    Clone this repository into the `~/ros/rap/Gruppe2` directory inside your `rap-jazzy` container.
    ```bash
    git clone <repository_url> ~/ros/rap/Gruppe2
    ```

2.  **Initialize the environment:**
    Source the `init.sh` script to set up the ROS2 workspace and install dependencies.
    ```bash
    source ~/ros/rap/Gruppe2/init.sh
    ```

## Dependencies

All required ROS 2 packages and Python libraries are automatically installed when you source the `init.sh` script. This script performs the following key dependency management tasks:

*   **ROS 2 Packages:**
    *   Clones the `m-explore-ros2` repository (which provides the `explore_lite` package for autonomous exploration).
    *   **Important:** The `map_merge` sub-package within `m-explore-ros2` is automatically removed by the `init.sh` script. This package is not required for the current setup and has compatibility issues with ROS 2 Jazzy.
    *   The `icclab_summit_xl` package, which provides the Summit XL robot simulation and Nav2 integration, is expected to be already installed in your ROS 2 workspace or will be resolved by `rosdep`.
*   **Python Packages:**
    *   Installs or upgrades necessary Python libraries for ROSA and the LLM interaction, including `jpl-rosa`, `langchain-ollama`, `langchain-core`, `pydantic`, `anthropic`, and `langchain-anthropic`.
*   **Gazebo Models:**
    *   Sets the `GZ_SIM_RESOURCE_PATH` environment variable to include the custom Gazebo models used in the simulation world.

The `init.sh` script also builds the Colcon workspace and runs `rosdep install` to ensure all system dependencies for the ROS 2 packages are met.

## Running the Simulation and Agent

1.  **Launch the Robot Simulation and Navigation:**
    This command starts the Gazebo simulation with the Summit XL robot and loads the navigation stack (Nav2). It can be launched in two modes:

    *   **With SLAM (for mapping new environments):**
        This mode enables SLAM (Simultaneous Localization and Mapping) and activates an autonomous exploration node. Use this mode when you want the robot to explore an unknown environment and create a new map.
        ```bash
        ros2 launch rosa_summit summit.launch.py slam:=True
        ```
        In this mode, you can use the `save_map` action (see "Available LLM Actions") to save the newly created map.

    *   **With a pre-existing map (for navigation in known environments):**
        This mode loads a default map (`maps/default.yaml`) and does not start SLAM or autonomous exploration. Use this mode when you have an existing map and want to navigate within it.
        ```bash
        ros2 launch rosa_summit summit.launch.py
        ```
        Or explicitly:
        ```bash
        ros2 launch rosa_summit summit.launch.py slam:=False
        ```

2.  **Run the LLM Agent:**
    In a new terminal (after sourcing `init.sh` or `~/colcon_ws/install/setup.bash`), run the ROSA LLM agent. This will allow you to interact with the robot using natural language.
    ```bash
    ros2 run rosa_summit rosa_summit
    ```

## Interacting with the Robot

Once the agent is running, you can type commands in the terminal where you launched `rosa_summit`. For example:
"drive forward at 0.5 meters per second"
"stop"
"start autonomous exploration"
"navigate to x 1.0 y 2.0"

## Simulation World

The simulation environment uses a modified version of the AWS Robomaker Small House World.
- **Original World:** [https://github.com/aws-robotics/aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- **Modifications:**
    - The world has been adapted from its original version. While a `ros2` branch exists in the original repository, further modifications were necessary to ensure compatibility with ROS 2 Jazzy.
    - Several objects that frequently obstructed the robot's path or caused navigation issues have been removed or repositioned.
    - The physics engine settings within the world file have been adjusted to improve the interaction and stability of the Summit XL robot.

## Available LLM Actions

The LLM can control the robot using the following actions:

*   **`send_vel(velocity: float)`**: Sets the forward velocity of the robot.
    *   Example: "drive forward at 0.2 meters per second"
*   **`stop()`**: Stops or halts the robot by setting its velocity to zero.
    *   Example: "stop the robot"
*   **`toggle_auto_exploration(resume_exploration: bool)`**: Starts or stops autonomous exploration.
    *   Example: "start exploring" or "stop exploring"
*   **`navigate_to_pose(x: float, y: float, z_orientation: float, w_orientation: float)`**: Moves the robot to an absolute position on the map using specified coordinates and orientation.
    *   Example: "go to position x 1.5 y -2.0 with orientation z 0.0 w 1.0"
*   **`navigate_relative(x: float, y: float, z_orientation: float, w_orientation: float)`**: Moves the robot relative to its current position.
    *   Example: "move 1 meter forward and 0.5 meters to the left"
*   **`save_map(map_name: str)`**: Saves the current map generated by SLAM.
    *   Example: "save the current map as my_house_map"
*   **`list_saved_maps()`**: Lists all previously saved maps.
    *   Example: "show me all saved maps"
*   **`get_location_names()`**: Returns a list of predefined location names.
    *   Example: "what are the known locations?"
*   **`navigate_to_location_by_name(location_name: str)`**: Moves the robot to a predefined named location.
    *   Example: "take me to the kitchen"

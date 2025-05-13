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

## Running the Simulation and Agent

1.  **Launch the Robot Simulation and Navigation:**
    This command starts the Gazebo simulation with the Summit XL robot, loads the navigation stack (Nav2), and enables SLAM.
    ```bash
    ros2 launch rosa_summit summit.launch.py
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

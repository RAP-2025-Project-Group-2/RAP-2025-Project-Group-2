from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.tools.ros2 import execute_ros_command
from rosa.prompts import RobotSystemPrompts
import os


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
def talk_to_user(message: str) -> str:
    """
    Sends a message to the user.

    :param message: the message to be sent
    """
    print("Sending message to user: %s" % message)
    return "Message sent: %s" % message


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
        tools=[send_vel, stop, talk_to_user],
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
                print(res)
            except Exception as e:
                print(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    agent.shutdown()
    print("Bye from rosa_summit.")


if __name__ == "__main__":
    main()

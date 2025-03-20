from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.tools.ros2 import execute_ros_command

@tool
def send_vel(velocity: float) -> str:
    """
    Sets the forward velocity of the robot
    
    :param velocity: the velocity at which the robot should move
    """
    # Your code here ...
    cmd = "ros2 topic pub -1 /summit/cmd_vel geometry_msgs/msg/Twist '{linear: {x: " + str(velocity) + ", y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}'"
    success, output = execute_ros_command(cmd)
    if success:
        return "Velocity set to %s" % velocity
    else:
        return "Failed setting velocity"

def main():
    print('Hi from rosa_summit.')
    ollama_llm = ChatOllama(
        model="hhao/qwen2.5-coder-tools:latest", #"gemma3:12b",  # or your preferred model
        temperature=0,
        num_ctx=32192,  # adjust based on your model's context window
        base_url="http://160.85.252.236:11434",
    )

    # Pass the LLM to ROSA
    agent = ROSA(ros_version=2, llm=ollama_llm, tools=[send_vel])

    while True:
        msg = input("Enter your request: ")
        res = agent.invoke(msg)
        print('Request sent')
        print(res)
    
    print('Bye from rosa_summit.')

if __name__ == '__main__':
    main()

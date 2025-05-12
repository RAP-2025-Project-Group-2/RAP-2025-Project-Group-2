export HOME=/home/ros
source /home/ros/.bashrc

ln -s /home/ros/rap/Gruppe2/ /home/ros/colcon_ws/src/

# Check and clone m-explore-ros2 if not present
EXPLORE_LITE_DIR="/home/ros/colcon_ws/src/m-explore-ros2"
if [ ! -d "$EXPLORE_LITE_DIR" ]; then
  echo "Cloning m-explore-ros2..."
  git clone https://github.com/robo-friends/m-explore-ros2.git "$EXPLORE_LITE_DIR"
else
  echo "m-explore-ros2 directory already exists."
fi

cd /home/ros/colcon_ws
colcon build --symlink-install
source install/setup.bash
sudo apt update -y
rosdep install --from-paths src --ignore-src -r -y
cd -
echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION**"


pip3 install jpl-rosa --break-system-packages
pip3 install langchain-ollama --upgrade --break-system-packages
pip3 install langchain-core --upgrade --break-system-packages
pip3 install pydantic --upgrade --break-system-packages
pip3 install anthropic --upgrade --break-system-packages
pip3 install langchain-anthropic --upgrade --break-system-packages
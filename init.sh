export HOME=/home/ros
source /home/ros/.bashrc

ln -s /home/ros/rap/Gruppe2/ /home/ros/colcon_ws/src/

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
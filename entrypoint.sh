set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 launch asump_bringup foxglove_demo.launch.py

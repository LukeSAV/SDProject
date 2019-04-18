#!/usr/bin/env bash
echo "Startup script running"
source /home/nvidia/.bashrc
bash -c "source /home/nvidia/workspace/SDProject/ros_ws/devel/setup.bash && roslaunch /home/nvidia/workspace/SDProject/ros_ws/launch/main.launch"

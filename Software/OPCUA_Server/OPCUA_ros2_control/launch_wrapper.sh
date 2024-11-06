#!/bin/bash

# Source the ROS 2 environment (update the path if needed)
#export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH


source /opt/ros/humble/setup.bash
source ~/.bashrc
source /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws/install/setup.bash
echo $LD_LIBRARY_PATH
# Source your workspace if you have one (update the path if needed)
# Uncomment the following line if you have a workspace that needs to be sourced
# source ~/your_ws/install/setup.bash
#export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
# Run the ros2 launch command with the provided package and launch file arguments
exec ros2 launch "$@"
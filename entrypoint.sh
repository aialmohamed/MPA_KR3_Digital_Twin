#!/bin/bash

# Fix ownership of the script
sudo groupadd robolab
echo "changing ownership of the directory to robolab ......"
sudo chown -R robolab:robolab /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/OPCUA_Server
sudo chown -R robolab:robolab /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env
echo "changed ownership of the directory to robolab"
sudo chmod +x /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/OPCUA_Server/OPCUA_shell_scripts/System_launch.sh
sudo chmod +x /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/OPCUA_Server/OPCUA_shell_scripts/System_kill.sh

## Run OPCUA Server 

source /opt/ros/humble/setup.bash


export CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH
## set boost :
export CMAKE_PREFIX_PATH=/home/robolab/boost_1_82:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/home/robolab/boost_1_82/lib:$LD_LIBRARY_PATH
export CPATH=/home/robolab/boost_1_82/include:$CPATH
source /opt/ros/humble/setup.bash


cd /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws
source ~/.bashrc
colcon build
source /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws/install/setup.bash
cd /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/OPCUA_Server
python3 OPCUA_server/opcua_server.py

# Execute the container's default CMD
exec "$@"

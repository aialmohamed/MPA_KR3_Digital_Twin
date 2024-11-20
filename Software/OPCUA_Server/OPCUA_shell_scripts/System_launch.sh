#!/bin/bash

PROJECT_DIR="/home/${USER}/MPA_Repo/MPA_KR3_Digital_Twin" # Change this to your project directory
SOFTWARE_DIR="${PROJECT_DIR}/Software"
KR3R540_WS="${SOFTWARE_DIR}/ROS2_Env/kr3r540_ws"
OPCUA_SERVER_DIR="${SOFTWARE_DIR}/OPCUA_Server"
SHELL_SCRIPTS_DIR="${OPCUA_SERVER_DIR}/OPCUA_shell_scripts"
LOGDIR="${SHELL_SCRIPTS_DIR}/log"

cd "${KR3R540_WS}" || exit
colcon build

source install/setup.bash
echo "source ${KR3R540_WS}/install/setup.bash"
source ~/.bashrc
source ./install/setup.bash
echo "sourcing ~/.bashrc"

ros2 launch kr3r540_bringup kr3r540_ign.launch.py > ${LOGDIR}/kr3r540_ign.log 2>&1 &
IGN_PID=$!
sleep 5
ros2 launch kr3r540_bringup kr3r540_ign_controller.launch.py > ${LOGDIR}/kr3r540_ign_controller.log 2>&1 &
IGN_CONTROLLER_PID=$!
sleep 5
ros2 launch kr3r540_bringup kr3r540_kinematics_server.launch.py > ${LOGDIR}/kr3r540_kinematics_server.log 2>&1 &
KINEMATICS_SERVER_PID=$!
sleep 5
ros2 launch kr3r540_bringup kr3r540_real.launch.py > ${LOGDIR}/kr3r540_real.log 2>&1 &
REAL_ROBOT_PID=$!
sleep 5

ros2 launch kr3r540_bringup kr3r540_real_controller.launch.py > ${LOGDIR}/kr3r540_real_controller.log 2>&1 &
REAL_CONTROLLER_PID=$!


echo "All processes launched successfully."
echo "${IGN_PID}" > ${LOGDIR}/kr3r540_pids
echo "${IGN_CONTROLLER_PID}" >> ${LOGDIR}/kr3r540_pids
echo "${KINEMATICS_SERVER_PID}" >> ${LOGDIR}/kr3r540_pids
echo "${REAL_ROBOT_PID}" >> ${LOGDIR}/kr3r540_pids
echo "${REAL_CONTROLLER_PID}" >> ${LOGDIR}/kr3r540_pids

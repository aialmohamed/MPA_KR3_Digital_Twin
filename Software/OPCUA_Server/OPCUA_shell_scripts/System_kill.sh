#!/bin/bash

echo "Stopping all ROS 2 and Ignition Gazebo processes..."

PROCESS_KEYWORDS=("ros2" "ign" "rviz2")

for keyword in "${PROCESS_KEYWORDS[@]}"; do

    pids=$(ps aux | grep -E "${keyword}" | grep -v grep | awk '{print $2}')
    
    for pid in $pids; do

        pgid=$(ps -o pgid= -p "$pid" | grep -o '[0-9]*')

        if [ -n "$pgid" ]; then
            echo "Stopping process tree with PGID: $pgid (keyword: $keyword)"
            kill -TERM -"$pgid" 2>/dev/null
        else
            echo "Could not determine PGID for PID: $pid (keyword: $keyword)"
        fi
    done
done
remaining_processes=$(ps aux | grep -E "ros2|ign" | grep -v grep)

if [ -n "$remaining_processes" ]; then
    echo "Some processes are still running:"
    echo "$remaining_processes"
    echo "Force killing remaining processes..."
    echo "$remaining_processes" | awk '{print $2}' | xargs sudo kill -9
else
    echo "All ROS 2 and Ignition Gazebo processes have been stopped."
fi

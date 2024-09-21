#!/bin/bash

# Function to check if Gazebo is installed
check_gazebo_installed() {
    if command -v gazebo &> /dev/null; then
        echo "Gazebo is already installed."
        return 0
    else
        return 1
    fi
}

# Function to install Gazebo
install_gazebo() {
    echo "Installing Gazebo..."
    
    # Update package lists
    sudo apt update
    
    # Install Gazebo packages for ROS 2 Humble
    sudo apt install -y ros-humble-gazebo*
    
    echo "Gazebo installation complete."
}

# Main script execution
if check_gazebo_installed; then
    exit 0
else
    install_gazebo
fi

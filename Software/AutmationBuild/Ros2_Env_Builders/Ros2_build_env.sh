# !/bin/bash



# Function to check if ROS 2 Humble is installed
check_ros2_installed() {
    if source /opt/ros/humble/setup.bash 2>/dev/null; then
        echo "ROS 2 Humble is already installed."
        return 0
    else
        return 1
    fi
}

# Function to check if Gazebo is installed
check_gazebo_installed() {
    if command -v gazebo &> /dev/null; then
        echo "Gazebo is already installed."
        return 0
    else
        return 1
    fi
}

install_ros2() {

# Set locales

echo "Setting up locales ..."
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
echo "locales setup is Done."

# Setup sources 
echo "Setting up sources ..."
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "sources setup is Done."

# Install ROS2 packages 
sudo apt -y update
sudo apt -y upgrade
# Check if ros2 exist : 

sudo apt install -y ros-humble-desktop
#source the setup.bash to the ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    echo "ROS 2 Humble installation complete."
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

# Ros2 Installer
if check_ros2_installed; then
    exit 0
else
    install_ros2
fi


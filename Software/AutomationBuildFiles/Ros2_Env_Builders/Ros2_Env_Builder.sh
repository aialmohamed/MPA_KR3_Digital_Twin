# !/bin/bash


# install yq and pip (always check those two)

if ! command -v yq &> /dev/null; then
  sudo snap install yq
else 
  echo "yq is already installed. Skipping installation..."
fi

if ! command -v pip &> /dev/null; then
  sudo apt-get -y install python3-pip
else
  echo "pip is already installed. Skipping installation..."
fi

# Check for boost

check_boost_asio_installed() {
    local boost_dir="$HOME/boost_1_82"
  if [ -d "$boost_dir" ]; then
    echo "Boost Asio is already installed at $boost_dir. Skipping installation..."
    return 0
  else
    echo "Boost Asio not found at $boost_dir. Installing Boost Asio..."
    return 1
  fi
}

# Function to check if ROS 2 Humble is installed
check_ros2_humble_installed() {
  dpkg -l | grep -q "ros-humble-desktop"
}
# Check collcon installed
check_colcon_installed() {
    command -v colcon &> /dev/null
}
# Check vscode installed

check_vscode_installed() {
  command -v code &> /dev/null
}

install_vscode() {
  echo "Checking if vscode is installed..."

    if check_vscode_installed; then
        echo "vscode is already installed. Skipping installation..."
    else
        echo "vscode not found. Installing vscode..."
        sudo apt update && sudo apt upgrade -y
        sudo apt install software-properties-common apt-transport-https wget -y
        wget -O- https://packages.microsoft.com/keys/microsoft.asc | sudo gpg — dearmor | sudo tee /usr/share/keyrings/vscode.gpg
        echo deb [arch=amd64 signed-by=/usr/share/keyrings/vscode.gpg] https://packages.microsoft.com/repos/vscode stable main | sudo tee /etc/apt/sources.list.d/vscode.list
        sudo apt update
        sudo apt install code
    fi
}

install_colcon() {

    echo "Checking if colcon is installed..."

    if check_colcon_installed; then
        echo "colcon is already installed. Skipping installation..."
    else
        echo "colcon not found. Installing colcon..."

        sudo apt update 
        sudo apt install -y python3-colcon-common-extensions
    fi
}


# Function to install ROS 2 Humble if it's not installed
install_ros2_humble() {
  echo "Checking if ROS 2 Humble is installed..."

  if check_ros2_humble_installed; then
    echo "ROS 2 Humble is already installed. Skipping installation..."
  else
    echo "ROS 2 Humble not found. Installing ROS 2 Humble..."

    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade

    # Install ROS 2 Humble desktop package
    sudo apt install -y ros-humble-desktop

#source the setup.bash to the ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    echo "ROS 2 Humble installation complete."

    # Install other required dependencies
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
  fi
}
install_boost_asio() {
  local boost_dir="$HOME/boost_1_82"
  if ! check_boost_asio_installed; then
    cd $HOME
    wget https://boostorg.jfrog.io/artifactory/main/release/1.82.0/source/boost_1_82_0.tar.gz
    tar -xvzf boost_1_82_0.tar.gz
    mv boost_1_82_0 boost_1_82
    cd boost_1_82
    ./bootstrap.sh --prefix=$boost_dir
    ./b2 install
    echo "Boost Asio installed successfully at $boost_dir."
  fi
}
# Function to remove ROS 2 Humble
remove_ros2_humble() {
  echo "Checking if ROS 2 Humble is installed..."

  if check_ros2_humble_installed; then
    echo "Removing ROS 2 Humble and related packages..."
   
    # Remove ROS 2 Humble and associated packages
    sudo apt-get remove --purge -y ros-humble-*
    sudo apt-get autoremove -y
    sudo apt-get clean
   
    # Remove ROS 2 repository and keys
    sudo rm /etc/apt/sources.list.d/ros2-latest.list
    sudo apt-key del "$(apt-key list | grep 'ROS' -A 1 | tail -n 1 | awk '{print $2}')"
   
    echo "ROS 2 Humble removed successfully."
  else
    echo "ROS 2 Humble is not installed."
  fi
}

# Function to install ROS packages from the YAML file
install_ros_packages() {
  echo "Reading package names from the YAML file..."
  packages=$(yq '.packages[]' ROS2_Packages.yaml)

  # Loop through the packages
  for package in $packages; do
    # Check if the package is installed
    if dpkg -l | grep -q "ros-humble-$package"; then
      echo "Package ros-humble-$package is already installed. Skipping..."
    else
      echo "Installing ros-humble-$package..."
      sudo apt-get install -y ros-humble-$package
    fi
  done
}

install_vscode_extensions() {
    echo "Reading extension names from the YAML file..."
    extensions=$(yq '.vscode_extensions[]' ROS2_Packages.yaml)

  # Loop through the packages
  for extension in $extensions; do
    # Check if the package is installed
    if ! code --list-extensions | grep -q "$extension"; then
      echo "Installing vscode-$extension..."
      code --install-extension "$extension"
    else
      echo "extension vscode-$extension is already installed. Skipping..."
    fi
  done

}

# Function to remove ROS packages from the YAML file
remove_ros_packages() {
  echo "Reading package names from the YAML file..."
  packages=$(yq '.packages[]' packages.yaml)

  # Loop through the packages
  for package in $packages; do
    # Check if the package is installed
    if dpkg -l | grep -q "ros-humble-$package"; then
      echo "Removing ros-humble-$package..."
      sudo apt-get remove --purge -y ros-humble-$package
    else
      echo "Package ros-humble-$package is not installed. Skipping..."
    fi
  done

  # Perform autoremove and clean up after removing packages
  sudo apt-get autoremove -y
  sudo apt-get clean
}

# Function to display help message
show_help() {
  echo "Usage: $0 {install|remove|help}"
  echo
  echo "Options:"
  echo "  install    Install ROS 2 Humble and the packages listed in packages.yaml"
  echo "  remove     Remove ROS 2 Humble and the packages listed in packages.yaml"
  echo "  help       Show this help message"
}

# Main script logic
if [[ "$1" == "install" ]]; then
  echo "Starting installation process..."

  # Step 1: Install ROS 2 Humble if necessary
  install_ros2_humble

  # Step 2: Install other ROS packages from the YAML file
  install_ros_packages

  # Step 3: Install colcon

  install_colcon

  # Step 4 : install vscode 
  install_vscode

  # Step 5 : install vscode_extensions
  install_vscode_extensions

  # Step 6 : install boost asio
  install_boost_asio

  echo "Installation process completed."

elif [[ "$1" == "remove" ]]; then
  echo "Starting removal process..."

  # Step 1: Remove ROS 2 Humble and associated packages
  remove_ros2_humble

  # Step 2: Remove other ROS packages from the YAML file
  remove_ros_packages

  echo "Removal process completed."

elif [[ "$1" == "help" ]] || [[ -z "$1" ]]; then
  show_help
else
  echo "Invalid option: $1"
  show_help
fi
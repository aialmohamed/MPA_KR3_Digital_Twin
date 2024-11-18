# Use an official ROS 2 image
FROM osrf/ros:humble-desktop

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install required dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    curl \
    python3 \
    python3-pip \
    git \
    build-essential \
    snapd \
    g++ \
    autotools-dev \
    libicu-dev \
    libbz2-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*
 
# Create a new user "robolab" with passwordless sudo
RUN useradd -rm -d /home/robolab -s /bin/bash -g root -G sudo -u 1001 robolab && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab && \
    chmod 0440 /etc/sudoers.d/robolab

USER robolab
WORKDIR /home/robolab


# Set Boost installation directory
ENV BOOST_DIR=/home/robolab/boost_1_82_0

# Download, extract, build, and install Boost in a single step
RUN wget https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz
RUN tar -xf boost_1_82_0.tar.gz
#RUN chmod +x /home/robolab/boost_1_82_0/b2
WORKDIR /home/robolab/boost_1_82_0
RUN /home/robolab/boost_1_82_0/bootstrap.sh --prefix=/home/robolab/boost_1_82
RUN ./b2
RUN ./b2 install


WORKDIR /home/robolab
RUN sudo apt-get update
RUN sudo apt-get install -y jq
RUN pip install yq
ENV PATH="$PATH:/home/robolab/.local/bin"


 
WORKDIR /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin
## install dependencies :

RUN sudo apt install -y ros-humble-ros2-control
RUN sudo apt install -y ros-humble-ros2-controllers
RUN sudo apt install -y ros-humble-joint-state-publisher-gui
RUN sudo apt install -y ros-humble-xacro
RUN sudo apt install -y ros-humble-ros-ign-bridge
RUN sudo apt install -y ros-humble-ign-ros2-control
RUN sudo apt install -y ros-humble-ros-gz

## install and setup colcon :
RUN sudo apt update
RUN sudo apt install -y python3-rosdep
RUN sudo apt install -y python3-colcon-common-extensions
RUN sudo apt install -y python3-ament-package

## isntall ignition 
RUN sudo apt-get update
RUN sudo apt-get install -y lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y ignition-fortress

COPY . .

#RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# remove old build :
WORKDIR /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws
RUN sudo rm -rf build/ install/ log/

RUN echo "export CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH" >> ~/.bashrc
## set boost :
RUN echo "export CMAKE_PREFIX_PATH=/home/robolab/boost_1_82:$CMAKE_PREFIX_PATH" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/home/robolab/boost_1_82/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "export CPATH=/home/robolab/boost_1_82/include:$CPATH" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
#RUN colcon build



EXPOSE 4840

# Default command
CMD ["bash"]

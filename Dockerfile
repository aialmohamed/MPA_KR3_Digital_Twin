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
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros-ign-bridge \
    ros-humble-ign-ros2-control \
    ros-humble-ros-gz \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-ament-package \
    lsb-release \
    gnupg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
 
# Create a new user "robolab" with passwordless sudo
RUN useradd -rm -d /home/robolab -s /bin/bash -g root -G sudo -u 1001 robolab && \
    echo "robolab ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robolab && \
    chmod 0440 /etc/sudoers.d/robolab


USER robolab
WORKDIR /home/robolab


# Set Boost installation directory
ENV BOOST_DIR=/home/robolab/boost_1_82_0

RUN sudo apt-get update &&\
    sudo apt-get install -y jq
RUN pip install yq
ENV PATH="$PATH:/home/robolab/.local/bin"

RUN wget https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz \
    && tar -xf boost_1_82_0.tar.gz \
    && cd boost_1_82_0 \
    && ./bootstrap.sh --prefix=/home/robolab/boost_1_82 \
    && ./b2 \
    && ./b2 install \
    && cd .. \
    && rm -rf boost_1_82_0 boost_1_82_0.tar.gz





 
WORKDIR /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin
    
RUN sudo apt-get update \
    && sudo apt-get install -y lsb-release gnupg wget \
    && sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y ignition-fortress \
    && sudo rm -rf /var/lib/apt/lists/*


## install asyncua

RUN pip install asyncua
COPY . .



#RUN rosdep init
WORKDIR /home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws

RUN rosdep update \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && sudo rm -rf build/ install/ log/



ENV USER=robolab

COPY entrypoint.sh /home/robolab/entrypoint.sh
WORKDIR /home/robolab
RUN sudo chmod +x /home/robolab/entrypoint.sh
EXPOSE 4840


ENTRYPOINT ["/home/robolab/entrypoint.sh"]

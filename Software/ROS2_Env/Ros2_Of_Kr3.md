# Ros2 for the Kuka KR3 


## Index
- [Ros2 for the Kuka KR3](#ros2-for-the-kuka-kr3)
  - [Index](#index)
  - [Concept](#concept)
    - [Understanding the ROS2 Controllers](#understanding-the-ros2-controllers)
      - [1. ROS 2 Controllers and Broadcasters](#1-ros-2-controllers-and-broadcasters)
      - [2. ROS 2 Robot Bring-Up](#2-ros-2-robot-bring-up)
      - [3. ROS 2 Controller Environment](#3-ros-2-controller-environment)
      - [4. ROS 2 Robot Description](#4-ros-2-robot-description)
      - [5. Hardware Components](#5-hardware-components)
      - [Data and Command Flow](#data-and-command-flow)
  - [Packages](#packages)
  - [Plugins](#plugins)
  - [Usage](#usage)
  - [Tests](#tests)


## Concept

### Understanding the ROS2 Controllers 

![Ros2_Controller_Concept](/Images/ROS2_Concept.jpg)

This diagram illustrates the architecture of the ROS 2 control system, which is used to interface between high-level robot controllers and low-level hardware components. The system is designed with modular components to allow for flexible control of robotic hardware and efficient management of resources. The architecture consists of several interconnected components, each with its distinct role in the control pipeline.

#### 1. ROS 2 Controllers and Broadcasters

The leftmost section of the diagram represents different controllers and broadcasters available in the ROS 2 control framework:

- **Forward Command Controller**: This controller accepts commands such as position, velocity, or effort, and directly forwards them to the robot’s hardware interface. It is typically used for direct control of actuators or other hardware components.

- **Joint Trajectory Controller**: This controller is used to execute pre-defined trajectories over a period. It is often utilized in applications like robotic arms, where smooth motion across multiple joints is essential.

- **Joint State Broadcaster**: This broadcaster publishes the state of each joint, such as position, velocity, and effort, based on the data read from the robot hardware. The joint states are typically used for visualization or feedback in higher-level applications.

#### 2. ROS 2 Robot Bring-Up

The ROS 2 Robot Bring-Up section contains configuration and launcher files that define and set up the controllers. The following activities take place during this phase:

- **Mapping of Joints to Controllers**: Using YAML configuration files, specific joints in the robot are mapped to different controllers based on the control strategy defined by the user.

- **Configuration of the Controller Manager**: The Controller Manager is configured with the necessary parameters, and launch files are set to initialize the controllers required for the robot’s operation.

#### 3. ROS 2 Controller Environment

The ROS 2 Controller Environment is responsible for managing the controllers and the resources associated with them:

- **Control Manager**: The Control Manager oversees the lifecycle of controllers, including loading, starting, stopping, and switching between controllers as necessary.

- **Resource Manager**: The Resource Manager initializes the hardware and manages resources such as joint definitions and hardware interfaces. It loads the URDF (Unified Robot Description Format) files, which define the physical and kinematic properties of the robot, and sets up the joint definitions accordingly.

#### 4. ROS 2 Robot Description

The ROS 2 Robot Description module provides the physical description of the robot in the form of URDF or XACRO files. These files define the structure of the robot, including links, joints, and their physical properties. The URDF and XACRO files are loaded by the Resource Manager to initialize the robot's configuration and map joints to controllers.

#### 5. Hardware Components

The Hardware Components section represents the physical robot hardware, which is interfaced through the **Robot Hardware Interface**. This interface acts as a bridge between the ROS 2 control framework and the actual hardware components:

- **Robot Hardware Interface**: This interface is responsible for sending command data to the hardware (e.g., position or velocity targets) and reading the state data (e.g., joint positions, velocities, or efforts) from the hardware. The state data is then fed back into the ROS 2 control framework for use by controllers and broadcasters.

#### Data and Command Flow

The flow of information and commands starts from the **ROS 2 Robot Description**, which defines the robot’s structure and joints. This information is loaded into the **Resource Manager**, which then initializes the hardware components through the **Robot Hardware Interface**. The **Control Manager** loads and configures the necessary controllers as specified in the bring-up configuration. Controllers then communicate with the **Robot Hardware Interface** to send position commands and receive state data. The state data is published by the **Joint State Broadcaster** and can be used for monitoring and control purposes.

Overall, this architecture provides a clear separation between the robot description, controller management, and hardware interface, making it highly modular and adaptable to various robotic systems and control strategies.

## Packages

## Plugins 

## Usage

## Tests
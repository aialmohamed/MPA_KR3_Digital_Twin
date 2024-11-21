# Digital Twin KR3

## Introduction and Motivation

This repository documents my Master Project at the **THU** under the supervision of **Prof. Lisa Ollinger**.

The project involves setting up a **digital twin** system for the **Kuka KR3 R540** robot. The key objectives are:

* Creating an interface between ROS and the robot via OPC UA
* Exploring potential use cases for the digital twin
> [!NOTE]
> duration of the project: 6 months from the beginning of September (from 01.09.2024 to 28.02.2025)

>[!NOTE]
> This repository is manged with this [jira](
>  https://ahmaedibrahim311-1720719768107.atlassian.net/jira/software/projects/DT/boards/2?atlOrigin=eyJpIjoiNjg4YjM5ZjIyYjM2NDUyYjlkYmYzN2RiZTRhMmNkNjUiLCJwIjoiaiJ9) project 

>[!NOTE]
>After the meeting with the professor on the 6th of August, we are using the KVP because it is easier to implement and we are not limiting the project to real-time (at least for now). 
>On the other hand, we have some suggestions for future development to improve the real-time aspect of the project, such as using a medium hardware like Revpi as a path for the RSI. so for this project we are using **KVP** (**KUKAVARPROXY**)

>[!NOTE]
> After the meeting on 06 August, we also came to the conclusion that we will implement the remote control use case.
>With this use case, we can then define a dashboard that acts as a control channel for the system, allowing us to control and monitor the robot(s) from a remote device.

>[!NOTE]
> The bug with binding the controller with the Simulation (solved using by adding use_sim_time flag in the launches.py of the movit_config)

>[!NOTE]
> The Interfaces between the real robot and ROS2 might require some kind of multiTasking/Threading due to the slow rate of the tcp connection in compare to the control loop of the ROS2
  
>[!NOTE]
> we are moving from normal gazebo  (gazebo classic) to ignition
## Index

- [Digital Twin KR3](#digital-twin-kr3)
  - [Introduction and Motivation](#introduction-and-motivation)
  - [Index](#index)
  - [System and development specification](#system-and-development-specification)
  - [To-do](#to-do)
    - [Setup and Repository](#setup-and-repository)
    - [Interface Development](#interface-development)
    - [Use Case and Evaluation](#use-case-and-evaluation)
  - [Approach](#approach)
    - [Motivation and Definitions](#motivation-and-definitions)
    - [Main System-Design](#main-system-design)
    - [Hardware Interface and connection](#hardware-interface-and-connection)
    - [Dashboard app](#dashboard-app)
  - [Setup](#setup)
  - [Usage](#usage)
    - [Build Docker](#build-docker)
      - [On Linux](#on-linux)
      - [On Windows :](#on-windows-)
    - [Run  on Linux](#run--on-linux)
    - [Run on Windows](#run-on-windows)
  - [Testing](#testing)
  - [Results](#results)



## System and development specification

|specification   | Version   | Note|
|----------------|-----------|-----|
| OS | Ubuntu Jammy (22.04) 64-bit | n/A |
| ROS2 |Humble Hawksbill |  n/A |
| Gazebo | Fortress | n/A|
| Robot | KuKa KR3 R540 | n/A|
| Robot controller | KRC4| n/A|
|KRC Windows version|WES7 4.0|n/A|
|KSS | 8.3 | n/A|
|cargo|1.82.0|n/A|


## To-do

### Setup and Repository
- [x] Create and set up a Git repository
- [x] Create the project's file system
- [x] Set up the system requirements
- [x] Create a concept for the props
- [x] Finalize the system setup (on my PC)
- [x] Set up the lab (lab's PC)
- [x] create connections by using UltraVCN and mount a driver to the Robot.
- [x] install the KUKAVARPROXY on the Lab PC and test it
- [x] (Optional) Automate the setup of the system

### Interface Development
- [x] Create the software for the interface between KR3 R540 and a local client (Experimental in c++)
- [x] Create the software for the interface between ROS2 and KR3 R540
- [ ] create an OPCUA Server
- [ ] create a start Dashboard (Web?)
- [ ] Test the interface with KR3 R540 (on other KR3 robots)
- [ ] Create the software for the ROS2 controllers (if required)

### Use Case and Evaluation
- [x] Define and document a use case / application of the digital twin
- [x] Evaluate the benefits and challenges of using a Digital Twin in the identified scenarios
- [x] Exchange insights with industry experts, particularly with Sven Völker


## Approach 

### Motivation and Definitions

To create a working digital twin of a robot with ROS2 and Gazebo, we first need to understand the basic concept of ROS2.

ROS2 is a middleware that allows us to control and monitor different types of robots by separating the hardware management layer, the control management layer and the application layer and then dynamically connecting them.

This allows us to constantly change the controllers and easily switch off the hardware, making it easier to use the hardware.

![ros control concept](/Images/ros2_controll_concept.png)


For the digital twin to work on the KuKa KR3, we need to develop the following 
* a hardware interface 
* a ROS2 controller layer
* a ROS2 Digital Twin Layer
* a ROS2-OPCUA-Bridge

This is only the side of the real robot. For the simulation or digital twin, we need to develop the simulation environment (Gazebo), so we need a dedicated BringUp ROS2 workspace for that.

Fortunately, the team from last semester has already taken care of the simulation (and the controllers), so all we have to do now is develop the interfaces between the hardware and the ROS side.

### Main System-Design



![sytem_init](/Images/DT_concept.jpg) 

### Hardware Interface and connection

>[!NOTE]
>According to this [research](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2561319), the main connection to the robot and the use of this connection as the basis of an OPCUA server is as follows.

| Aspect                   | KVP   (KUKAVARPROXY)                                      | RSI          (RobotSensorInterface)                                 | Implications                                      |
|--------------------------|---------------------------------------------|----------------------------------------------|---------------------------------------------------|
| **Time-Delay**           | Average time-delay of 8.75ms                | Average time-delay of 4.0ms                  | RSI has a lower time-delay, suitable for faster response times. |
| **Communication Protocols** | Uses TCP (Transmission Control Protocol)  | Uses UDP (User Datagram Protocol)            | UDP is faster but less reliable; TCP is more reliable but can have higher latency. |
| **Real-Time Requirements** | Less stringent real-time requirements      | Harder real-time requirements                | RSI provides faster responses but may be less stable. |
| **System Requirements**   | Can run in the background on the controller | Requires running on a Linux machine          | KVP offers more flexibility and resource efficiency. |
| **Reliability**           | More reliable, stable background operation  | Higher likelihood of breakdowns              | KVP is more robust for long-term stability.       |
| **Functional Requirements** | Requires a specific program for position updates | Requires a specific program for position updates | Both need dedicated resources on the controller for position updates. |
| **Advantages**            | - Higher reliability<br>- Flexibility<br>- Ease of use | - Lower latency<br>- Better real-time performance | KVP is better for general reliability and flexibility, RSI is better for low-latency applications. |
| **Drawbacks**             | - Higher latency                            | - Higher risk of breakdowns<br>- More resource intensive | Trade-off between reliability and performance depending on use case. |

### Dashboard app



>[!NOTE]
On this [Gitrepo](https://github.com/ImtsSrl/openshowvar/blob/master/resources/kukavar.txt) you can find all the global variables that we can change with KUKAVARPROXY , we can create our own.
## Setup 

>[!NOTE]
> Add a step by step tutorial for remote desktop UltraVNC and Sharing files on network locations
> [This Video](https://www.youtube.com/watch?v=4pO9Pvz3nBQ)


>[!NOTE]
> Add a reference to the use guid 

>[!NOTE]
> if KUKAVARPROXY dose not work check this out : https://github.com/ImtsSrl/KUKAVARPROXY/issues/18
## Usage 

### Build Docker 


#### On Linux
    
    cd /path/to/repo
    sudo docker build -it kr3r540_digital_twin .

#### On Windows :

    cd path/to/repo
    docker build -t digital_twin . 

### Run  on Linux
make sure that the x11 have access to docker : 

    xhost +local:docker

Running the system container :

    sudo docker run -it --rm     --gpus all     --net=host     -e NVIDIA_VISIBLE_DEVICES=all     -e NVIDIA_DRIVER_CAPABILITIES=all     -e DISPLAY=$DISPLAY     -v /tmp/.X11-unix:/tmp/.X11-unix   --memory-reservation=1g     kr3r540_digital_twin

### Run on Windows 

      docker run -it -p 4840:4840  -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER=/mnt/wslg/PulseServer --gpus all  kr3r540_digital_twin

## Testing 

## Results
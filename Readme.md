# Digital Twin KR3

## Introduction and Motivation

> A digital twin is a virtual representation of an object or system designed to reflect a physical object accurately. It spans the object's lifecycle, is updated from real-time data and uses simulation, machine learning and reasoning to help make decisions.

This repository documents my Master Project at the **THU** under the supervision of **Prof. Lisa Ollinger**.

The project involves setting up a **digital twin** system for the **Kuka KR3 R540** robot. The key objectives are:

* Creating an interface between ROS and the robot via OPC UA
* Exploring potential use cases for the digital twin
> [!NOTE]
> duration of the project: 6 months from the beginning of September (from 01.09.2024 to 28.02.2025)

>[!NOTE]
> This repository is manged with this [jira](
>  https://ahmaedibrahim311-1720719768107.atlassian.net/jira/software/projects/DT/boards/2?atlOrigin=eyJpIjoiNjg4YjM5ZjIyYjM2NDUyYjlkYmYzN2RiZTRhMmNkNjUiLCJwIjoiaiJ9) project 

## Index

- [Digital Twin KR3](#digital-twin-kr3)
  - [Introduction and Motivation](#introduction-and-motivation)
  - [Index](#index)
  - [System and development specification](#system-and-development-specification)
  - [To-do](#to-do)
    - [Setup and Repository](#setup-and-repository)
    - [Interface Development](#interface-development)
    - [Use Case and Evaluation](#use-case-and-evaluation)
    - [Digital Twin Application](#digital-twin-application)
  - [Approach](#approach)
    - [Conceptualisation](#conceptualisation)
    - [Main System-Design](#main-system-design)
    - [Hardware Interface and connection](#hardware-interface-and-connection)
  - [Setup](#setup)
  - [Software/s](#softwares)
  - [Testing](#testing)
  - [Results](#results)



## System and development specification

|specification   | Version   | Note|
|----------------|-----------|-----|
| OS | Ubuntu Jammy (22.04) 64-bit | n/A |
| ROS2 |Humble Hawksbill |  n/A |
| Gazebo | Fortress | n/A|
| Robot | KuKa KR3 R540 | n/A|


## To-do

### Setup and Repository
- [x] Create and set up a Git repository
- [ ] Clearfiy OPCUA usage ? connection to kuka is via rsi or opcua ????
- [ ] Create the project's file system
- [ ] Set up the system requirements
- [ ] Create a concept for the props
- [ ] Finalise the system setup (on my PC)
- [ ] Set up the lab (lab's PC)
- [ ] (Optional) Automate the setup of the system

### Interface Development
- [ ] Create the software for the interface between ROS2 and KR3 R540
- [ ] Test the interface with KR3 R540 (on other KR3 robots)
- [ ] Create the software for the ROS2 controllers (if required)

### Use Case and Evaluation
- [ ] Define and document a use case / application of the digital twin
- [ ] Evaluate the benefits and challenges of using a Digital Twin in the identified scenarios
- [ ] Exchange insights with industry experts, particularly with Sven Völker

### Digital Twin Application
- [ ] Design and develop a software architecture that connects the virtual and real robots
- [ ] Implement the communication between the virtual simulation and the real robot using OPC-UA
- [ ] Test and validate the Digital Twin application through synchronised movements and data comparisons between simulation and reality

## Approach 

### Conceptualisation

To create a working digital twin of a robot with ROS2 and Gazebo, we first need to understand the basic concept of ROS2.

ROS2 is a middleware that allows us to control and monitor different types of robots by separating the hardware management layer, the control management layer and the application layer and then dynamically connecting them.

This allows us to constantly change the controllers and easily switch off the hardware, making it easier to use the hardware.

![ros control concept](/Images/ros2_controll_concept.png)


For the digital twin to work on the KuKa KR3, we need to develop the following 
* a hardware interface 
* a ROS2 interface
* a ROS2 controller 
* a ROS2-OPCUA-Bridge

This is only the side of the real robot. For the simulation or digital twin, we need to develop the simulation environment (Gazebo), so we need a dedicated BringUp ROS2 workspace for that.

Fortunately, the team from last semester has already taken care of the simulation (and the controllers), so all we have to do now is develop the interfaces between the hardware and the ROS side.

### Main System-Design



![sytem_init](/Images/Sytem_main_idea.png) 

### Hardware Interface and connection

>[!NOTE]
>According to this [research](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2561319), the main connection to the robot and the use of this connection as the basis of an OPCUA server is as follows.

| Aspect                   | KVP                                         | RSI                                           | Implications                                      |
|--------------------------|---------------------------------------------|----------------------------------------------|---------------------------------------------------|
| **Time-Delay**           | Average time-delay of 8.75ms                | Average time-delay of 4.0ms                  | RSI has a lower time-delay, suitable for faster response times. |
| **Communication Protocols** | Uses TCP (Transmission Control Protocol)  | Uses UDP (User Datagram Protocol)            | UDP is faster but less reliable; TCP is more reliable but can have higher latency. |
| **Real-Time Requirements** | Less stringent real-time requirements      | Harder real-time requirements                | RSI provides faster responses but may be less stable. |
| **System Requirements**   | Can run in the background on the controller | Requires running on a Linux machine          | KVP offers more flexibility and resource efficiency. |
| **Reliability**           | More reliable, stable background operation  | Higher likelihood of breakdowns              | KVP is more robust for long-term stability.       |
| **Functional Requirements** | Requires a specific program for position updates | Requires a specific program for position updates | Both need dedicated resources on the controller for position updates. |
| **Advantages**            | - Higher reliability<br>- Flexibility<br>- Ease of use | - Lower latency<br>- Better real-time performance | KVP is better for general reliability and flexibility, RSI is better for low-latency applications. |
| **Drawbacks**             | - Higher latency                            | - Higher risk of breakdowns<br>- More resource intensive | Trade-off between reliability and performance depending on use case. |




## Setup 

## Software/s

## Testing 

## Results
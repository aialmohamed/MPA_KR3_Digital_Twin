# Digital Twin KR3

## Introduction and Motivation

> A digital twin is a virtual representation of an object or system designed to reflect a physical object accurately. It spans the object's lifecycle, is updated from real-time data and uses simulation, machine learning and reasoning to help make decisions.

This repository documents my Master Project at the **THU** under the supervision of **Prof. Lisa Ollinger**.

The project involves setting up a **digital twin** system for the **Kuka KR3 R540** robot. The key objectives are:

* Creating an interface between ROS and the robot
* Developing an OPC UA server to synchronize the real robot with the simulated robot
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

![ros control concept](../MPA_KR3_Digital_Twin/Images/ros2_controll_concept.png)


For the digital twin to work on the KuKa KR3, we need to develop the following 
* a hardware interface 
* a ROS2 interface
* a ROS2 controller 

This is only the side of the real robot. For the simulation or digital twin, we need to develop the simulation environment (Gazebo), so we need a dedicated BringUp ROS2 workspace for that.

Fortunately, the team from last semester has already taken care of the simulation (and the controllers), so all we have to do now is develop the interfaces between the hardware and the ROS side.

### Hardware Interface and connection

| Feature                     | RSI (Robot Sensor Interface)          | KVP (KUKA Variable Protocol) | EKI (Ethernet KRL Interface)    |
|-----------------------------|---------------------------------------|------------------------------|---------------------------------|
| **Purpose**                 | Real-time control and sensor integration | Non-real-time monitoring and control | Flexible communication using XML |
| **Communication**           | Real-time Ethernet                    | TCP/IP                       | TCP/IP                          |
| **Real-Time Support**       | Yes                                   | No                           | Limited real-time support       |
| **Typical Use Cases**       | Adaptive control, precision tasks, advanced robotics | Monitoring, configuration, data logging | Custom protocols, flexible data exchange |
| **Ease of Use**             | Complex                               | Simple                       | Moderate                        |
| **Integration**             | Requires real-time systems expertise  | Easy to integrate with various environments | Requires knowledge of XML and KRL |
| **Update Frequency**        | High frequency (millisecond range)    | Lower frequency              | Moderate frequency              |
| **Data Format**             | XML-based configuration               | Plain text commands          | XML-based messages              |
| **Implementation Complexity**| High                                  | Low                          | Moderate                        |
| **Computational Resources** | High                                  | Low                          | Moderate                        |
| **Advantages**              | Real-time performance, advanced control | Simplicity, ease of implementation | Flexibility, custom data formats  |
| **Disadvantages**           | Complexity, resource-intensive        | Non-real-time                | Requires XML and KRL knowledge  |
| **Typical Applications**    | Research, high-precision manufacturing, robotics R&D | Basic monitoring and control, configuration | Custom integrations, flexible data handling |

## Setup 

## Software/s

## Testing 

## Results
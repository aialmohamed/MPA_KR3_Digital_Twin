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

## Setup 

## Software/s

## Testing 

## Results
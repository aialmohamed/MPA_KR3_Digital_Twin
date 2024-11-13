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
| Robot controller | KRC4| n/A|
|JDK(Java Development Kit)|21|n/A|
|JavaFX|21 or 22|n/A|
|Gradle|8.5|n/A|
|KRC Windows version|WES7 4.0|n/A|
|KSS | 8.3 | n/A|


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

## Setup 

## Software/s

## Testing 

## Results
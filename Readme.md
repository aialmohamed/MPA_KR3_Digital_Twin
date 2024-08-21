# Digital Twin KR3

## Introduction and Motivation

> digital representation of a unique asset (product, machine, service,
> product service system or other intangible asset), that alters
> its properties, condition and behaviour by means of models,
> information and data

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
>After the meeting with the professor on the 6th of August, we are using the CIP because it is easier to implement and we are not limiting the project to real-time (at least for now). 
>On the other hand, we have some suggestions for future development to improve the real-time aspect of the project, such as using a medium hardware like Respi as a path for the RSI. so for this project we are using **KVP** (**KUKAVARPROXY**)

>[!NOTE]
> After the meeting on 06 August, we also came to the conclusion that we will implement the remote control use case.
>With this use case, we can then define a dashboard that acts as a control channel for the system, allowing us to control and monitor the robot(s) from a remote device.
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
    - [Motiviation and Defnitions](#motiviation-and-defnitions)
    - [Main System-Design](#main-system-design)
    - [Hardware Interface and connection](#hardware-interface-and-connection)
    - [Dashboard app](#dashboard-app)
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
| .NET | 8.0| n/A |
| Avalonia | latest| n/A|


## To-do

### Setup and Repository
- [x] Create and set up a Git repository
- [x] Create the project's file system
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
- [x] Define and document a use case / application of the digital twin
- [x] Evaluate the benefits and challenges of using a Digital Twin in the identified scenarios
- [ ] Exchange insights with industry experts, particularly with Sven Völker

### Digital Twin Application
- [ ] Design and develop a software architecture that connects the virtual and real robots
- [ ] Implement the communication between the virtual simulation and the real robot using OPC-UA
- [ ] Test and validate the Digital Twin application through synchronised movements and data comparisons between simulation and reality

## Approach 

### Motiviation and Defnitions

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

| Framework/Toolkit    | Description                                                                 | Supported Languages     | Platform Support            |
|----------------------|-----------------------------------------------------------------------------|-------------------------|-----------------------------|
| Electron             | Framework for building cross-platform desktop apps using web technologies.  | JavaScript, HTML, CSS    | Windows, macOS, Linux       |
| Avalonia             | .NET UI framework for building cross-platform applications.                 | C#                       | Windows, macOS, Linux       |
| .NET MAUI            | Cross-platform framework for building native mobile and desktop apps.       | C#                       | Windows, macOS              |
| WPF (Windows Presentation Foundation) | UI framework for building Windows desktop applications.     | C#, XAML                 | Windows                     |
| GTK                  | Toolkit for creating graphical user interfaces.                             | C, Python, Vala          | Windows, macOS, Linux       |
| Qt                   | Cross-platform application development framework.                           | C++, QML                 | Windows, macOS, Linux       |
| WinUI                | Latest UI framework for Windows desktop apps.                               | C#, C++, XAML            | Windows                     |
| JavaFX               | Platform for building rich internet applications with Java.                 | Java, FXML               | Windows, macOS, Linux       |
| SwiftUI              | UI toolkit by Apple for building user interfaces across all Apple devices.  | Swift                    | macOS                       |
| Tauri                | Lightweight framework for building cross-platform desktop apps.             | Rust, JavaScript, HTML   | Windows, macOS, Linux       |
| PyQt                 | Python binding for the Qt toolkit.                                          | Python                   | Windows, macOS, Linux       |
| Lazarus              | Open-source cross-platform IDE similar to Delphi.                           | Object Pascal            | Windows, macOS, Linux       |
| Uno Platform         | Cross-platform framework for building single-codebase applications.         | C#                       | Windows, macOS, Linux       |
| Xamarin.Forms        | Framework for building cross-platform mobile and desktop apps.              | C#                       | Windows, macOS              |


The Dashboard desktop app shall be devloped using Avalonia framwork , due to it being a corssplatform and easy to wrap for a web interface or mobileapp interface.
The Dashboard shall be defined in the system requierment file , where also is the use cases of this app is defined .
For Backend we are using Mysql. 
>[!NOTE]
The Name of the project (app) in avalonia should be short otherwies the previewer wont work
## Setup 

>[!NOTE]
> Add a step by step tutorial for remote desktop UltraVNC and Sharing files on network locations
> [This Video](https://www.youtube.com/watch?v=4pO9Pvz3nBQ)

## Software/s

## Testing 

## Results
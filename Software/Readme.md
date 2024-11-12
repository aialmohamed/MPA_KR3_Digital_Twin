# MPA Software

## Index
- [MPA Software](#mpa-software)
  - [Index](#index)
  - [Experimental Software](#experimental-software)
    - [CppOpenShowVar](#cppopenshowvar)
  - [ROS2\_Env](#ros2_env)
  - [OPCUA\_Server](#opcua_server)
  - [Dashboard](#dashboard)
  - [Automation system Build](#automation-system-build)


## Experimental Software  

### [CppOpenShowVar](../Software/Experimental/CppOpenShowVarClient/CppOpenShowVar.md)
a Cpp Library using boost.asio to create a communication with the Kukavarproxy server on the Robot .
This Library is also used to implement the **Hardware-Interface** of the kr3r540 ros2 robot.
This library contains mainly a tcp layer (lowest layer) that uses a asynchronous tcp communication with the ability to serialize and deserialize the read/write and response messages from and to the robot.

I have created some test cases (unit/integration tests) (focused for the serialization and deserialization) but doing some more tests would help improve and optimize this library.


## [ROS2_Env](../Software/ROS2_Env/Ros2_Of_Kr3.md)

This is the core of the Digital Twin system , here you can find all the ros2 packages that I built in order to create the Digital Twin **(using Gazebo Ignition)** of the kr3r540.
The packages include all the needed urdf files of the robot , the controllers of the simulated robot and the real robot, hardware interface layer of the real robot, inverse kinematics action server using KDL library, ros2 user defined messages .digital twin nodes and the ros2 launchers.

## OPCUA_Server

## Dashboard
TBD (Before 15.12.2024)
## Automation system Build
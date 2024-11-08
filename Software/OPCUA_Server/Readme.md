# OPCUA and Ros2 Server 


## Index

- [OPCUA and Ros2 Server](#opcua-and-ros2-server)
  - [Index](#index)
  - [Concept](#concept)
    - [Top Level Sequence](#top-level-sequence)
  - [Classes and Structure](#classes-and-structure)
  - [Sequences and connectivity](#sequences-and-connectivity)
  - [Usage](#usage)
  - [Tests](#tests)


## Concept 

![Concept](../../Images/Opcua-ros2_concept.jpg)

The main idea is to build ros capabilities within an OPCUA server , so that Ros2 is some kind of a Plugin of an OPCUA server.

After building the needed Ros2 packages in the **[ros2 kr3r540-Core](../ROS2_Env/Ros2_Of_Kr3.md)** , the ros2 layer of the system is ready to be used by the opcua server.

Now when we create  a uamethod to communicate with the **ros2 kr3r540-core** ,the method must have a ros2 node binding in the background .
The Ros2 node in the OPCUA server environment should always run on a separate thread due to it being mostly a sub/pub  node that need to be always alive until the OPCUA-Client stops it .

After triggering the Ros2 node the Ros2 node subs/pubs to a Ros2 topic of the **ros2 kr43r540 Core** so that the server can interact with the robot (real and simulation)
After that the Ros2 Node in the OPCUA server environment can grab the values that we need and publish them to the OPCUA server as a OPCUA-variable which is continually updated until the OPCUA-Client stops it.

### Top Level Sequence

![Sequence](../../Images/opcua_ros2_top_level_seq.jpg)

Here we can see how the different layers of the system interacts with each other.

A User using the **Dashboard** triggers a method that is **ua-method** on the opcua client side , this then triggers an **ua-method** on the opcua server side , that triggers a Ros2 node (inside the server) to start the communication with the **ros2 Kr3r540 core** that controls the real and simulated robot .
On the other hand the Robot (real or simulation) can send the data through the **ros2 kr3r540 core** over to the **opcua server** by exposing the **Ros2 node** that is responsible to update an **OPCUA server variable** after that the opcua client simply read the variable and updates it.

## Classes and Structure

## Sequences and connectivity  

## Usage

## Tests
# CppOpenShowVar a KukaVarProxy client concept



## Index
- [CppOpenShowVar a KukaVarProxy client concept](#cppopenshowvar-a-kukavarproxy-client-concept)
  - [Index](#index)
  - [Concept](#concept)
    - [Overview of Layers](#overview-of-layers)
      - [1. **Application Layer**](#1-application-layer)
      - [2. **KukaClient Layer**](#2-kukaclient-layer)
      - [3. **TCPClient Layer**](#3-tcpclient-layer)
      - [4. **KRC4 Robot Controller**](#4-krc4-robot-controller)
    - [Detailed Flow](#detailed-flow)
      - [**Write Command Flow**](#write-command-flow)
      - [**Read Command Flow**](#read-command-flow)
    - [Callbacks and Notification](#callbacks-and-notification)
  - [Classes](#classes)
    - [1. **TCPClient**](#1-tcpclient)
    - [2. **KukaVarProxyClient**](#2-kukavarproxyclient)
    - [3. **Message** (Abstract Class)](#3-message-abstract-class)
    - [4. **WriteMessage**](#4-writemessage)
    - [5. **ReadMessage**](#5-readmessage)
    - [6. **ResponseMessage**](#6-responsemessage)
  - [Usage](#usage)
  - [Tests](#tests)

## Concept 

The main concept is to run a cpp tcp/ip client on the host machine .

![OpenShowVarConcept](/Images/openShowVarAsync_Concept.jpg)
This diagram illustrates the asynchronous communication flow between a user issuing commands and a KUKA robot controller (KRC4) through the KUKAVarProxyClient. The system is structured into three main layers: Application Layer, KukaClient Layer, and TCPClient Layer, each handling specific responsibilities in the async communication process.

### Overview of Layers

#### 1. **Application Layer**
- The **user** sends either a **write** or **read** command to the robot.
- For a **write command**, the user invokes `KukaClient.writeAsync()` with the desired variable (e.g., `$OV_PRO`) and the value (e.g., `50`).
- For a **read command**, the user invokes `KukaClient.readAsync()` with the variable name (e.g., `$OV_PRO`).
- After sending the command, **callbacks** (`writeCallback` or `readCallback`) are triggered to notify the user of the result.

#### 2. **KukaClient Layer**
- The **KukaClient** acts as an interface between the application layer and the **TCPClient**.
- For a **write operation**, it constructs a **`WriteMessage`** (to change a robot variable) and serializes it into a binary format, which is then sent via **TCPClient**.
- For a **read operation**, it constructs a **`ReadMessage`** (to request a variable value) and similarly serializes it for transmission.
- Upon receiving the robot's response, **`ResponseMessage`** is used to **deserialize** the binary response, extracting the result of the write operation or the value of the requested variable in a read operation.

#### 3. **TCPClient Layer**
- The **TCPClient** handles the asynchronous TCP communication with the robot via **KUKAVARPROXY**.
- It sends the serialized **read** or **write** commands to the robot and waits for the response asynchronously.
- Once the robot's response is received, it forwards the response data back to the **KukaClient** for processing.

#### 4. **KRC4 Robot Controller**
- The **KRC4 controller** receives the **write** or **read** commands from **KUKAVARPROXY** and processes them. 
  - For a **write command**, it updates the variable on the robot and sends a confirmation response back to the client.
  - For a **read command**, it retrieves the value of the requested variable and sends it back in the response.

### Detailed Flow

#### **Write Command Flow**
1. **User** triggers a write command: `KukaClient.writeAsync("$OV_PRO", "50")`.
2. **`WriteMessage`** serializes the command into binary format (HEX) and sends it to the robot via **TCPClient**.
3. The **robot** processes the command and sends a response indicating whether the operation was successful.
4. **`ResponseMessage`** deserializes the response to check if the write operation succeeded.
5. The **`writeCallback`** in the **Application Layer** is invoked, notifying the user of success or failure.

#### **Read Command Flow**
1. **User** triggers a read command: `KukaClient.readAsync("$OV_PRO")`.
2. **`ReadMessage`** serializes the command into binary format (HEX) and sends it to the robot via **TCPClient**.
3. The **robot** retrieves the value of the requested variable and sends it back in a response.
4. **`ResponseMessage`** deserializes the response, extracting the value of the variable.
5. The **`readCallback`** in the **Application Layer** is invoked, notifying the user of the variable’s value.

### Callbacks and Notification
- The **callbacks** in the **Application Layer** (`writeCallback` or `readCallback`) are triggered after the robot's response is processed. They notify the user whether the **write operation** succeeded or return the variable's value for a **read operation**.

This architecture ensures non-blocking, efficient communication with the robot, allowing the system to remain responsive while waiting for robot responses.
## Classes

![CppOpenShowVar_Classes](/Images/CppOpenShowVarClient_Classes.png)


This class diagram outlines the main components for sending and receiving **read** and **write** commands to the KUKA robot using asynchronous TCP communication. The classes handle constructing the messages, serializing them into binary format, and deserializing the robot’s responses.

### 1. **TCPClient**
The `TCPClient` class manages the low-level asynchronous TCP communication with the KUKA robot. It is responsible for sending and receiving binary data using **Boost.Asio**.
- **Role in the Concept Diagram**: This class is responsible for the actual TCP connection (sending and receiving commands/responses). It bridges the **KukaClient Layer** and the **KRC4 Robot**.

### 2. **KukaVarProxyClient**
The `KukaVarProxyClient` class acts as the main interface between the **Application Layer** and the robot. It provides high-level methods for sending **read** and **write** commands asynchronously by using the `TCPClient` for communication.
- **Role in the Concept Diagram**: It handles **`writeAsync()`** and **`readAsync()`** operations, sending messages through the **TCPClient** and managing callbacks to notify the **Application Layer**.

### 3. **Message** (Abstract Class)
The `Message` class is a base class that provides the structure for both read and write commands. It defines the common fields (message ID, content length, and mode) and ensures serialization and deserialization of messages.
- **Role in the Concept Diagram**: This abstract class forms the foundation for creating specific **ReadMessage** and **WriteMessage** objects that can be serialized into binary format and sent to the robot.

### 4. **WriteMessage**
The `WriteMessage` class is responsible for constructing the write command message to update a variable on the KUKA robot (e.g., setting `$OV_PRO` to `50`). It serializes the message into binary format.
- **Role in the Concept Diagram**: Used by the **KukaVarProxyClient** to send **write commands** to the robot through **TCPClient**.

### 5. **ReadMessage**
The `ReadMessage` class constructs the read command message to request the value of a variable on the robot (e.g., reading `$OV_PRO`). It serializes the request into binary format.
- **Role in the Concept Diagram**: Used by the **KukaVarProxyClient** to send **read commands** through **TCPClient** and retrieve variable values.

### 6. **ResponseMessage**
The `ResponseMessage` class deserializes the robot’s response (whether from a read or write command). It processes the response, extracts the variable value (in case of a read), or checks whether the operation was successful (for a write).
- **Role in the Concept Diagram**: It handles deserializing the robot's response and invokes the corresponding **callback** in the **Application Layer** (either `writeCallback` or `readCallback`).

## Usage
n/a
## Tests

This library has been tested (look under tests). maybe more tests needs to be done .
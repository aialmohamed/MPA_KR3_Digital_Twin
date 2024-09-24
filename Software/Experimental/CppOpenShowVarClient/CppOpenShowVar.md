# CppOpenShowVar a KukaVarProxy client concept



## Index
- [CppOpenShowVar a KukaVarProxy client concept](#cppopenshowvar-a-kukavarproxy-client-concept)
  - [Index](#index)
  - [Concept](#concept)
      - [1. **Application Layer**](#1-application-layer)
      - [2. **KukaClient Layer**](#2-kukaclient-layer)
      - [3. **TCPClient Layer**](#3-tcpclient-layer)
      - [4. **Robot (KRC4)**](#4-robot-krc4)
    - [**Flow Summary**:](#flow-summary)
  - [Classes](#classes)
  - [Usage](#usage)

## Concept 

The main concept is to run a cpp tcp/ip client on the host machine .

![OpenShowVarConcept](/Images/openShowVarAsync_Concept.jpg)
This diagram illustrates the asynchronous communication flow between a user issuing commands and a KUKA robot controller (KRC4) through the KUKAVarProxyClient. The system is structured into three main layers: Application Layer, KukaClient Layer, and TCPClient Layer, each handling specific responsibilities in the async communication process.
#### 1. **Application Layer**
- The **Application Layer** represents the user's interface with the system. Here, the user sends commands to read or write values from/to the KUKA robot. For example:
  - The user issues a write command to change the robot's speed (`KukaClient.writeAsync("$OV_PRO", "50")`).
  - Similarly, the user can request the current robot speed using a read command (`KukaClient.readAsync("$OV_PRO")`).
- **Callbacks** (`writeCallback` and `readCallback`) are provided when making these async calls. These callbacks will be executed when the robot’s response is received. 
- Once the callback is invoked, the user is **notified** about the outcome of the operation (e.g., successful write or the value read).

#### 2. **KukaClient Layer**
- The **KukaClient Layer** acts as a middleman between the Application Layer and the low-level TCP communication. It handles the construction of **WriteMessage** and **ReadMessage** objects, which are serialized to a byte stream in HEX format and sent to the robot.
- When a command like `writeAsync()` or `readAsync()` is called, the KukaClient layer:
  - Serializes the command into the appropriate format.
  - Passes the serialized message to the **TCPClient Layer** for asynchronous transmission.
- Once the robot's response is received (via the TCPClient Layer), the **callback functions** (e.g., `writeCallback`, `readCallback`) are invoked to process the response (e.g., deserializing the response, checking for errors, etc.).
- This layer ensures that operations are non-blocking by using async methods and triggering callbacks only when responses arrive.

#### 3. **TCPClient Layer**
- The **TCPClient Layer** handles the low-level networking operations. It sends serialized commands (both read and write) to the KRC4 robot controller over TCP asynchronously.
- The TCPClient asynchronously:
  - **Sends the serialized write command** to the robot and waits for the **write response** (e.g., acknowledgment that the robot speed has been updated).
  - **Sends the serialized read command** to the robot and waits for the **read response** (e.g., the current value of the `$OV_PRO` variable).
- Once the response (either read or write) is received and deserialized, the TCPClient triggers the corresponding **callback** in the KukaClient Layer, which then notifies the user of the result.

#### 4. **Robot (KRC4)**
- The KUKA robot controller (KRC4) receives the **write commands** (e.g., to set the speed to 50%) or **read commands** (e.g., to retrieve the current robot speed) from the **TCPClient Layer**.
- It then processes the command and sends back the appropriate response (either a confirmation of the write operation or the value requested in a read operation).

---

### **Flow Summary**:
1. **User Interaction**: The user sends an asynchronous command using `writeAsync()` or `readAsync()` with a **callback** function to handle the response.
2. **Message Serialization**: In the **KukaClient Layer**, the write or read command is serialized into HEX format and passed to the TCPClient for asynchronous transmission.
3. **Async Send and Receive**: The **TCPClient Layer** sends the serialized command to the robot and listens for a response asynchronously, without blocking the control loop.
4. **Callback Execution**: Once the response is received and deserialized, the callback function (e.g., `writeCallback` or `readCallback`) is invoked to handle the response.
5. **User Notification**: The user is notified via the callback when the robot’s response is processed, indicating the success or failure of the operation.

This architecture ensures non-blocking, efficient communication with the robot, allowing the system to remain responsive while waiting for robot responses.
## Classes

## Usage
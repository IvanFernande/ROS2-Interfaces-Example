# ROS2 Interfaces Example

In this example, we will create two nodes, one being a fictitious battery that sends its status through a service to a fictitious led panel, which will be updated according to its status. As it is a service, the battery node will collect the feedback that the panel was updated correctly.

In turn, the node that manipulates the LED panel will publish a message with the status of the LEDs it has, being a 0 if it is off, and 1 if it is on.

For example, if the panel has 4 leds, of which the first and third are on, the panel status will be something like: `[1,0,1,0]`.

<p align="center">
  <img src="https://github.com/user-attachments/assets/5729c644-2567-4e64-b8f6-744c18976914" width="600">
</p>

To explain this example, we will first introduce what interfaces in ros are, explain the interfaces used in this example, as well as the development of the nodes and interfaces to run the example.

# Interfaces in ROS2

In ROS2, interfaces are the way nodes communicate with each other by passing data. These interfaces are defined in terms of message types, services and actions, which are fundamental to the ROS2 architecture. There are three types of interfaces: messages, services and actions, but in this example, the first two will be used:

## Messages:
**Messages** in ROS2 are used for asynchronous communication between nodes. They are transmitted through topics and allow a node to publish data while others subscribe to it.

- **Structure**: A message in ROS2 is defined in an `.msg` file containing the specification of the data to be transmitted. Messages may contain several fields of primitive types such as `int32`, `float32`, `string`, or even other nested message. In this example we can see how the message `LedStateArray.msg` collects the information in `int64[]` format, which is a 64-bit array of integers representing the states of various LEDs. Each position in the array corresponds to a specific LED and its numeric value can represent different states of the LED (e.g., on or off).

<p align="center">
  <img src="https://github.com/user-attachments/assets/0f5c8a3a-5858-47b8-b990-baca5b19a746" width="300">
</p>

- **Posting** and **subscribing** messages:
  - Publisher: A node publishes messages to a topic.
  - Subscriber: A node listens to a topic to receive messages posted by other nodes.
- **Example of use**:
  - Topic: `"led_states"`
  - Message type: `LedStateArray.msg`
  - Publisher: A node that reports the current status of the LED panel.
  - Subscriber: A node that uses the status for something not specified.

## Services:
A **service** in ROS2 provides a mechanism for synchronous communication between two nodes. It works in a request-response style, where one node requests an operation and another node performs the task and responds with the result.

- **Structure**: A service is defined in an `.srv` file containing two parts:
  - Request: The data that is sent to the node to request something.
  - Response: The data that the node responds to after processing the request.
  In this example, the `SetLed.srv` service requests two parameters: `int64 led_number`, which identifies the LED to control, and `int64 state`, which specifies the new state of the LED (e.g., 0 for off and 1 for on). In response, the server returns a boolean value `bool success`, indicating whether the operation was successful (true) or not (false).
<p align="center">
  <img src="https://github.com/user-attachments/assets/8c9772cf-f573-4aaf-bc13-5af40df4fcd8" width="300">
</p>

- **Service call** and **response**:
  - Client: The node that sends the request to the service.
  - Server: The node that receives the request, performs the task and sends the response.
  
- **Example of use**:
  - Service: `"set_led"`
  - Service type: `SetLed.srv`
  - Client: A node that requests to change the status of each LED in the panel.
  - Server: A node that processes the request and change the status of each LED in the panel.

# Code Implementation and Results
Now, having theoretically introduced what interfaces are, let's go to the example mentioned at the beginning.

## Led Panel Node

This project implements a ROS2 node called **LedPanelNode** that manages the state of an LED panel. The node allows you to control and monitor the status of the LEDs using a service and a topic, respectively.

### Node Description

The **LedPanelNode** performs two main functions:
1. **Publishing LED states**: It uses a topic to publish the current states of the LEDs as an array.
2. **Service to modify the state of an LED**: It exposes a service that allows other nodes to change the state of a specific LED, either turning it on or off.

#### Main Functions:

##### 1. Publishing LED States
The node periodically publishes the state of the LEDs via the `led_states` topic with a message of type `LedStateArray`. Initially, the state of all LEDs is `[0, 0, 0]` (off). A timer is used to publish the states every 4 seconds.

##### 2. Service to Change the State of an LED
The node offers a service called `set_led`, which allows changing the state of an individual LED. The service takes two parameters:
- `led_number`: The number of the LED to modify (starting from 1).
- `state`: The new state of the LED, where `0` means off and `1` means on.

The node validates that the requested LED number exists and that the provided state is valid (`0` or `1`). If the inputs are correct, the LED state is updated, and a successful response (`success = True`) is returned. Otherwise, a failure response (`success = False`) is sent.





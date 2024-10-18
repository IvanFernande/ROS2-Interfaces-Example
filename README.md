# ROS2 Interfaces Example

In this example, we will create two nodes, one being a fictitious battery that sends its status through a service to a fictitious led panel, which will be updated according to its status. As it is a service, the battery node will collect the feedback that the panel was updated correctly.

In turn, the node that manipulates the LED panel will publish a message with the status of the LEDs it has, being a 0 if it is off, and 1 if it is on.

For example, if the panel has 4 leds, of which the first and third are on, the panel status will be something like: `[1,0,1,0]`.

<p align="center">
  <img src="https://github.com/user-attachments/assets/cc91b3f4-ab15-4af4-bcf8-989b4bef2170" width="500">
</p>


To explain this example, we will first introduce what interfaces in ros are, explain the interfaces used in this example, as well as the development of the nodes and interfaces to run the example.

# Interfaces in ROS2

In ROS2, interfaces are the way nodes communicate with each other by passing data. These interfaces are defined in terms of message types, services and actions, which are fundamental to the ROS2 architecture. There are three types of interfaces: messages, services and actions, but in this example, the first two will be used:

## Messages:
**Messages** in ROS2 are used for asynchronous communication between nodes. They are transmitted through topics and allow a node to publish data while others subscribe to it.

- **Structure**: A message in ROS2 is defined in an `.msg` file containing the specification of the data to be transmitted. Messages may contain several fields of primitive types such as `int32`, `float32`, `string`, or even other nested message. In this example we can see how the message `HardwareStatus.msg` collects the information in `int64` format of the temperature of some hardware as well as in `bool` format if some engine is ready to run and in `string` format a debug message.
<p align="center">
  <img src="https://github.com/user-attachments/assets/1eb4af1e-32aa-42d2-8692-deccd461cde5" width="200">
</p>

- **Posting** and **subscribing** messages:
  - Publisher: A node publishes messages to a topic.
  - Subscriber: A node listens to a topic to receive messages posted by other nodes.
- **Example of use**:
  - Topic: `"hardware_status"`
  - Message type: `HardwareStatus.msg`
  - Publisher: A node that reports the current status of the hardware.
  - Subscriber: A node that uses the status for something not specified.

## Services:
A **service** in ROS2 provides a mechanism for synchronous communication between two nodes. It works in a request-response style, where one node requests an operation and another node performs the task and responds with the result.

- **Structure**: A service is defined in an `.srv` file containing two parts:
  - Request: The data that is sent to the node to request something.
  - Response: The data that the node responds to after processing the request.
  In this example, we can see how the `ComputeRectangleArea.srv` service asks the client to send in `float64` format the length and width data, so that the service returns in `float64` format the area to the client.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c1042d13-3696-40a2-bc17-521a2d0e5f6b" width="200">
</p>

- **Service call** and **response**:
  - Client: The node that sends the request to the service.
  - Server: The node that receives the request, performs the task and sends the response.
  
- **Example of use**:
  - Service: `"compute_rectangle_area"`
  - Service type: `ComputeRectangleArea.srv`
  - Client: A node that requests to change the position of the robot.
  - Server: A node that processes the request and moves the robot to the new position.

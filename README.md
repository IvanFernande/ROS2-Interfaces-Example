# ROS2 Interfaces Example

In ROS2, interfaces are the way nodes communicate with each other by passing data. These interfaces are defined in terms of message types, services and actions, which are fundamental to the ROS2 architecture. There are three types of interfaces: messages, services and actions, but in this example, the first two will be used:

## Messages:
Messages in ROS2 are used for asynchronous communication between nodes. They are transmitted through topics and allow a node to publish data while others subscribe to it.

- **Structure**: A message in ROS2 is defined in an `.msg` file containing the specification of the data to be transmitted. Messages may contain several fields of primitive types such as `int32`, `float32`, `string`, or even other nested message.
- **Posting** and *subscribing* messages:
  - Publisher: A node publishes messages to a topic.
  - Subscriber: A node listens to a topic to receive messages posted by other nodes.
- **Example of use**:
  - Topic: /robot/position
  - Message type: Position.msg
  - Publisher: A node that reports the current position of the robot.
  - Subscriber: A node that uses the position for navigation or visualisation.

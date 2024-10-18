# ROS2 Interfaces Example

In ROS2, interfaces are the way nodes communicate with each other by passing data. These interfaces are defined in terms of message types, services and actions, which are fundamental to the ROS2 architecture. There are three types of interfaces: messages, services and actions, but in this example, the first two will be used:

## Messages:
Messages in ROS2 are used for asynchronous communication between nodes. They are transmitted through topics and allow a node to publish data while others subscribe to it.

  · Structure: A message is composed of several fields that can be of basic types (integers, strings, booleans) or compound types (arrays, other messages).
  · Example: The message `my_interface/msg/LedStateArray` has several fields such as the different leds and the state of each one, for example, off or on.

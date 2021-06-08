This folder contains some examples of possible interactions between ROS 2 and Lingua Franca.

To install the latest version of ROS 2, refer to (https://index.ros.org/doc/ros2/Installation/).

The examples have been tested on *Foxy Fitzroy* on Ubuntu 20.04 LTS.

To see instructions on how to set up each example, refer to the files themselves. Here is an
overview of the current examples:

- BasicROS: A basic two reactor example that publishes and subscribes to 'topic' and sends a "Hello, world!" string.

- PTIDES-ROS: Similar to BasicROS but the content of the message is an Int64 which is the current logical time of
              the sender + 25 msec. If 25 msec is sufficiently larger, the timestamp will be preserved between the
              sender reactor and the receiver reactor. This is an implementation of a poor-person's Ptides.

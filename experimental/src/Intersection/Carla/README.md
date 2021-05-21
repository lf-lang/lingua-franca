This is a very early example of an LF program communicating with Carla through the Carla ROS Bridge.

To get from the CarlaIntersection.lf program to a ROS node, simply run:
```
./build-ROS-node.sh
```

The Lingua Franca program talks to the Carla simulator via the `carla-ros-bridge`, available [here](https://github.com/carla-simulator/ros-bridge). Documentation on how to install it can be found [here](https://github.com/carla-simulator/ros-bridge/blob/master/docs/ros_installation_ros2.md).

Currently, this example is only tested with the one-car EGO example provided by the `carla-ros-bridge`, which after sourcing the appropriate terminal, can be executed as:

```
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

Assuming that Carla is running, and after sourcing the terminal:

```
source lf_carla/install/setup.bash
```

you should be able to run the example:

```
ros2 run lf_carla lf_carla_publisher
```




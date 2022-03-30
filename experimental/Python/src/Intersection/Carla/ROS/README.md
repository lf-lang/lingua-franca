# ROS implementation of the Carla Intersection Experiment

To run the experiment, run CARLA:
```bash
~$ ../run-carla.sh
```

Then, use `colcon` to build both `carla_intersection` and `carla_intersection_msgs`:
```bash
~$ cd carla_intersection_msgs
~$ colcon build
~$ cd ..
~$ cd carla_intersection
~$ colcon build
```

Finally, launch the ros program using the launch file:
```bash
~$ ros2 launch launch/intersection_demo.launch.py
```


## Noteworthy Observations / Comparisons with Lingua Franca Implementation

- Goal
    - Preventing the vehicle from issueing another request before the RSU replies with a grant.
- Implementation
    - ROS 
        - An additional flag is needed to track a pending request.
    - LF (Centralized Coordination)
        - Vehicle's logical time cannot advance before receiving a grant, so the goal is achieved by default.

### Introduction
The _ros_control_ interface for the _MuJoCo_ simulator.
The code parses a given model and register a control interface for each _slide_ or _hinge_ joint.
The _ros_control_ effort based controllers could be then loaded as shown in example _start_simple_robot.launch_.
It provides trajectory based interface e.g. for MoveIt.  


#### Run tests
All tests should pass if the installation was successful.
```
colcon build --cmake-target tests && colcon test --packages-select  mujoco_ros_control && colcon test-result --all
...
Summary: X tests, 0 errors, 0 failures, 0 skipped
where X >= 2
```


### Simple 1 DOF Example:

```
roslaunch mujoco_ros_control start_simple_robot.launch
roslaunch mujoco_ros_control rviz.launch
```

### Simulation of Kuka lwr/lbr robot provided in separate repository:

<https://version.aalto.fi/gitlab/robotic_manipulation/kuka_mujoco>
 

### Resources
- https://version.aalto.fi/gitlab/robotic_manipulation/mujoco_ros_control
- http://www.mujoco.org/
- https://colcon.readthedocs.io/

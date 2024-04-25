# lumi_testbed

The set of core ROS packages for lumi robot. Contains URDF description,  moveit configuration, mujoco configuration.

## Installation
Prerequisites:
```sh
sudo apt install ros-melodic-libfranka ros-melodic-franka-ros
```

## Launch robot
Simulated robot without MuJoCo and MoveIt:
```sh
roslaunch lumi_description show.launch 
```

Simulated robot with MuJoCo and MoveIt:
```sh
roslaunch lumi_mujoco simulation.launch
```

Real robot:
```sh
TBD. 
```

# Drone Simulation Project
A simulation environment for multiple drones using the simulator _Webots_ and the framework _ROS2 (Robot Operating System)_ for implementing control logics.

## Table of Contents
- [Installation](#installation)
- [Usage]($usage)
- [Architecture](#architecture)

<h2 id="installation">Installation</h2>
This project requires the instalation of ROS2 to work. The recommended version is ROS2 Humble, which can be installed by 
following the instructions on ROS2 Humble official website:

[ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

<h2 id="usage">Usage</h2>

### Activate environment
In order to run the simulation, first you need to activate ROS2 on your machine by running the following command

```bash
source /opt/ros/humble/setup.bash
```

Then, make sure you're inside the project workspace and run
```bash
source install/local_setup.bash
```

### Run simulation
Before running the simulation, run the following command, which will build ROS2 nodes 
```bash
colcon build
```

Then, in order to run the simulation, run the following command
```bash
ros2 launch mavic_simulation robot_launch.py
```
This will execute the launcher of the project, install Webots if you don't have it already installed, ask how many drones are desired in the simulation and, finally, open the Webots window of the simulaton.

### Control drones
Once the simulation has been launched successfully, run the following command to run the controller for a drone, changing the **ID** in **teleop_twist_keyboard_Mavic_ID** and **cmd_vel_Mavic_ID** to the index of the drone you want to control.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_Mavic_ID -r cmd_vel:=cmd_vel_Mavic_ID
```
Drones are counted from left to right, which means the leftmost drone is the drone with ID 1 and the rightmost drone is the drone with ID n, being n the amount of drones in the simulation.

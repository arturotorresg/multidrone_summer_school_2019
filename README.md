# Multidrone Summer School 2019
*v0.0.1*

## Overview
This is the repository for the exercises of the workshop on "Drones with ROS and Gazebo simulations". It includes the necessary code to complete and perform a simulation with several drones following a car and pointing their cameras to this car.

## Dependencies

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [grvc-ual](https://github.com/grvcTeam/grvc-ual) at tag [v2.2](https://github.com/grvcTeam/grvc-ual/tree/v2.2)
* [PX4 Firmware](https://github.com/PX4/Firmware) at tag [v1.7.3](https://github.com/PX4/Firmware/tree/v1.7.3)

## Installation

### Install dependencies

* Install grvc-ual and PX4 Firmware following [these instructions](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual).
* Hector gazebo plugins - `sudo apt-get install ros-kinetic-hector-gazebo-plugins`

### Clone to workspace and compile
```
cd ~/multidrone_ws/src
git clone https://github.com/arturotorresg/multidrone_summer_school_2019
catkin build
source ~/.bashrc
```

## Folder Structure
- drcsim_gazebo_plugins: DRC XP900 vehicle Gazebo plugins
- drcsim_gazebo_ros_plugins: DRC XP900 vehicle ROS plugins
- drcsim_model_resources: DRC XP900 model resources
- summer_school_exercises: main package with example node and launch file

## Launch Files
TBD

## Nodes
- follow_vehicle.py - For the first exercise
- gimbal_control.py - To continue

## Licences
MIT License
# Multidrone Summer School 2019 - Drones with ROS and Gazebo simulations

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
- summer_school_exercises: main package where you will work on

## Exercises
The exercises are designed such that their complexity increases. Maybe we will not have enough time to finish them all during the workshop, but you can finish them later at home.

### Part A: Playing with UAL
We start with simple exercises to know how UAL works, how to simulate several drones and how to command them.

1. **Start a simulation** - `roslaunch uav_abstraction_layer test_server.launch`
2. **Run UAL state monitor** - `rosrun uav_abstraction_layer state_monit.py`
3. **Check topics and services** - `rostopic list` | `rosservice list`
4. **Command the drone through command line** - Call the take-off, land and go to waypoint services, send position and velocity commands through topics.
5. **Command the drone through key_teleop node** - `rosrun ual_teleop key_teleop.py`
6. **Run a simulation with 3 drones** - `roslaunch uav_abstraction_layer test_server.launch multi:=true ns_prefix:=drone_`
7. **Repeat the previous steps with the 3 drones** - Observe the namespacing.

### Part B: Follow a moving target and shooting it with the camera
Here is where you actually use the UAL to follow a moving target, an autonomous car, and also where you can implement a gimbal controller to point the camera at this car at the same time.

1. **Check the code structure** - Look at the sample code and check the available data.
2. **Follow the vehicle with one drone** - Add the necessary code to command the drone movement so it follows the vehicle with a specific relative position.
3. **Point the camera** - Implement the controller to move the gimbal so the camera points to the vehicle.
4. **Follow the vehicle with three drones in a fixed formation** - Create a new launch file to simulate 3 drones and to run one follow_vehicle node and one gimbal_control node for each drone.
5. **Perform a dynamic shot type** - Now the relative position to the vehicle should be dynamic. Change the parametrization of the relative position. Try performing a FLYBY, in which the drone starts behind the vehicle with a certain distance in the Y axis, then overtakes it and finalise the shot in front of the vehicle.
6. **Add a duration to the shot** - Now you need to care about the timing, and try to perform the shot within a certain duration.

### Part C: Complex missions and coordination from a GCS
This is the advanced part. Once the drones are ready to perform dynamic shot types, it is time to coordinate a mission from the Ground Control Station (GCS).

1. **Implement a ROS service to receive shot parameters** - The callback should be in the follow_vehicle node.
2. **Implement a GCS node to send the shots** - The GCS node should call the service of each drone so they start doing the shots.

# Table of contents
* [Objective](#objective)
* [Installations and dependencies](#installations-and-dependencies)
* [Usage](#usage)
* [Development process](#development-process)
* [Problems found during development](#problems-found-during-development)

# Objective

The aim of this repository is to show the step-by-step creation of a bicycle-like robot, its challenges, problems, solutions and lessons learned.

# Installations and dependencies

The first step was to determine a robotics framework that provided all the necessary tools for implementation, as well as having supporting documentation and open source development.

The framework chosen was ROS Noetic, as it was the most recent in terms of development for the ROS1 platform. It was also necessary to install an operating system compatible with the framework. In this case, I installed Ubuntu 20.04 LTS (Focal Fossa), as it is also an open-source operating system.

### Ubuntu:
[Linux Ubuntu 20.04.6 LTS Focal Fossa image download link.](https://releases.ubuntu.com/focal/)

### ROS Framework:
[ROS1 Noetic installation guide for Ubuntu.](http://wiki.ros.org/noetic/Installation/Ubuntu)

It was installed the full Desktop version of ROS, and all dependencies of Python 3.

After the installations, we need to install the packages responsible for the Gazebo dependencies, motion controllers and joint controllers. I used the following terminal lines for this:

### All gazebo dependencies packages:
```
sudo apt install ros-noetic-gazebo*
```
### Joint controllers messages and plugins:
```
sudo apt install ros-noetic-joint*
```
### Controllers basic plugins for gazebo:
```
sudo apt install ros-noetic-ros-control*
```
### Plugins for Velocity and Effort transmissions and controllers:
```
sudo apt install ros-noetic-controller*
```
### Ackermann controller and messages for movement:
```
sudo apt install ros-noetic-ackermann-*
```
### Create a new package and clone this repo
Go to the desired folder and type on terminal to create a workspace:
```
mkdir -p catkin_ws/src
```
Go to the source folder of the workspace:
```
cd catkin_ws/src/
```
#### Clone this repository in this location using your SSH key or HTTPS

Return to the main workspace folder:
```
cd ..
```
Build the packages:
```
catkin_make
```
Source the workspace so that ROS can identify your new packages:
```
source devel/setup.bash
```
#### Please stick to this workspace sourced folder so that the following steps can work.

# Usage

### If you want to see the joints and meshes of the vehicle, use the description launcher:

```
roslaunch ugv_assign_description description.launch
```
![](/imgs/description.gif)

### If you want to move the robot with RQT commands in a empty Gazebo World:

```
roslaunch ugv_assign_gazebo gazebo_empty_rqt.launch
```
![](/imgs/gazebo_empty.gif)

### If you want to move the robot with RQT commands in a Desert Gazebo World:

```
roslaunch ugv_assign_gazebo gazebo_desert_rqt.launch
```
![](/imgs/gazebo_desert.gif)

### Sand

```
roslaunch ugv_assign_gazebo gazebo_desert_sand.launch
```
Keep in mind that sand is still a work in progress, both in size and mechanical properties.
![](/imgs/gazebo_sand.png)

### Other launchers

#### Empty World

```
roslaunch ugv_assign_gazebo gazebo_empty.launch
```

```
roslaunch ugv_assign_gazebo gazebo_empty_sand_rqt.launch
```

```
roslaunch ugv_assign_gazebo gazebo_empty_sand.launch
```

#### Desert

```
roslaunch ugv_assign_gazebo gazebo_desert.launch
```

# Development process

## Research and articles

Since the intention was to create a bicycle-like robot, I had to research which controller would best represent the movement of a mountain bike.

I used the following 3 articles and the Steer Bot robot as a reference to understand how the controller works, what the implementation problems would be, how the joints work together, and if there is a Gazebo plugin already implemented.

### [An adaptive preview path tracker for off-road autonomous driving.](https://www.researchgate.net/publication/261345739_An_adaptive_preview_path_tracker_for_off-road_autonomous_driving)
This is article explained in many details how the Ackermann control system can work, and how the transition from a 4-wheel car to a 2-wheel bicycle could function.
![](/imgs/ackermann-bike.png)

Picture Source: LI, Xiaohui; SUN, Zhenping; CHEN, Qingyang; LIU, Daxue.
### [A BICYCLE ROBOT: PART 2 SYSTEM IMPLEMENTATION.](https://www.researchgate.net/publication/266879573_A_BICYCLE_ROBOT_PART_2_SYSTEM_IMPLEMENTATION)

### [Development and control of a bicycle robot based on steering and pendulum balancing.](https://www.sciencedirect.com/science/article/abs/pii/S0957415820300660)

The main learning from the above articles was how to develop some parts with the purpose of balancing the robot. Both use small support wheels with minimal drag to prevent the bike from falling over if the robot receives external forces on its side. This is a problem that is showed again in later sections of this README.

![](/imgs/robobike.png)

Picture Source: MONYAKUL, Veerapol; SOORAKSA, Pitikhate; T. Uthairat; S. Kaopratum.

![](/imgs/robobike2.jpg)

Picture Source: SEEKHAO, Pongsakorn; TUNGPIMOLRUT, Kanokvate; PARNICHKIN, Manukid.

### [Steer Bot Github Repository.](https://github.com/srmainwaring/steer_bot/tree/master)

This repository contains an ackermann controller robot that can be used as an example, and its controllers configurations.

![](https://raw.githubusercontent.com/wiki/srmainwaring/steer_bot/images/steer_bot_gazebo.png)

Picture Source: MAINWARING, Rhys.

### [Ackerman Steering Controller ROS Package Summary](http://wiki.ros.org/ackermann_steering_controller)

The main source of the controller. Available in ROS Wiki. Contains enough examples on how to setup this controller in your robot.

## Robot construction

The second step was to develop the robot. To do this, I needed a mountain bike model as a reference. Based on recommendations made by users on bicycle forums, videos on the subject and the availability of its dimensions and values, my initial design was the based on Bike 4060 Z LT XT, from SCOR. 

![](/imgs/4060.jpg)

Picture Source: SCOR.

Of course, this couldn't be translated 100%, but it was a great start. After that, I needed a 3D modeling software. I decided on OnShape: an online cad software system, that can be used to develop incredible models using the browser.

![](/imgs/Onshape.png)

The following gifs are showing how the robot came to be, body and wheels.

#### Wheels

![](/imgs/wheel_ons.gif)

I used the following video as references for modeling:

#### [ Modeling Bi Cycle Wheel | By Cycle Wheel Modeling Tutorial - SolidWorks 2020 by ERUDIRE PLUS](https://www.youtube.com/watch?v=ywsbOVoe3cE)

#### Body

![](/imgs/bike_ons.gif)

I used the following video as references for modeling:

#### [ Full Suspension Mountain Bike Assembly and Test - Episode 11 - SOLIDWORKS LIVE Design by SOLIDWORKS](https://www.youtube.com/watch?v=7lOS-IhD0AY)


#### Camera

The camera used as example was a ZED 2i.

![](/imgs/zed.png)

Picture Source: Stereo Labs.

#### [ZED 2i Datasheet](https://www.stereolabs.com/assets/datasheets/zed-2i-datasheet-feb2022.pdf)


## Gazebo plugins and URDF

The code referenced here is related to this file: [bicycle_robot.urdf](/ugv_assign_description/urdf/bicycle_robot.urdf).

### Links and Joints

The links and joints were created based on these references:

[ROS Wiki - urdf](http://wiki.ros.org/urdf)

[ROS Wiki - urdf/XML/joint](http://wiki.ros.org/urdf/XML/joint)

The main points to consider are: 

- Most of the meshes and links are connected by a fixed joint, which means that they must not move in relation to each other. Some parts are complex, which means that they were developed individually and connected later.

- The robot's handlebar has a rotation limit of -0.5 rad to 0.5 rad. The type of joint that best defines this limitation is the joint of revolution, where limits can be set for rotation, effort and torque.

- The wheels can rotate freely around their axis. Continuous joints are great for this functionality, and accept drag and friction values for realism in the wheel.

- The suspensions use prismatic joints. These are used to move linearly their joints in relation to an axe. They were used to raise and lower a wheel when transitioning to more difficult terrain.

- The robot have in total a mass of approximately 207.08 kg. The values that were placed in the inertia matrix were based on the values suggested by OnShape. Below is an example of the inertia matrix of the wheel, if it was made out of Steel 1010. The values of mass and inertia were changed in such a way that the robot would not break due to its own center of mass interactions.

![](/imgs/inertia.png)

### Plugins 

Using the Gazebo plugins as reference ([Gazebo plugins in ROS](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)), I was able to create topic and associate them in the URDF.

#### Camera

You can subscribe to both topics below to watch the images of the cameras.

```/bicycle_robot/camera/left/image_raw```

```/bicycle_robot/camera/right/image_raw```

![](/imgs/camera.gif)

#### IMU

You can subscribe to topic below to watch the value of acceleration and velocity of the bicycle.

```/bicycle_robot/imu```

![](/imgs/imu.png)

#### LiDAR

You can subscribe to topic below to watch the value of the rays colliding on a object.

```/velodyne_points```

Velodyne rays from the robot:

![](/imgs/velodyne.png)

Velodyne values from above rays:

![](/imgs/velodyne_term.png)

#### Joints and Controller

The published joint states can determine the position and velocity of the wheels and suspension joint of the bike.

![](/imgs/joints_terminal1.png)

```/bicycle_robot/joint_states```

#### Sand
I used the "populate" feature of the World SDF to generate a number of particles with random distribution to create a carpet of sand. The example below is 200 particles.
![](/imgs/sand.gif)


# Problems found during development

### Sideways Fall - Solved

#### Problem: 
![](/imgs/FirstProblem-BikeFalls.gif)

That was the first problem found in development of the robot. Even tough the center of mass is stable and all inertias are very similar to the collisions shape, eventually the robot is falling sideways. This could be attributed to the irregular shape of the wheels. The solution of this problem was the two small wheels in the center of mass of the robot.

### Lack of supension - Unsolved

#### Problem: 
Even though all 4 wheels have damp and friction values, the lack of suspension makes it very difficult for the bike to move over difficult terrain. 

Right now, this model is working with a value of friction so high that the own weight of the robot could even move.

The gif below shows what happens if there a decrease in friction values at the joint. The own weight of the bicycle can push the prismatic joint. In addition, there is no possibility of returning to the initial point implemented.
![](/imgs/suspension_prismatic_problem.gif)

The gif below shows the type of problem that could be avoided if the prismatic joint in the front was effective. To climb this small hill, the robot should put the weight on the front wheel in such a way that the suspension was activated and the two largest wheels maintained contact with the ground.
![](/imgs/SecondProblem-GainsOnlyonBack.gif)

Another problem that could be avoided is the reduction in drag and fricction imposed on the rear wheel, since the ackermann controller sends gain values only to the robot's rear axis. The gif below shows the amount of drag needed to break the inertia of the hill, and what happens when the momentum of the robot is kept.
![](/imgs/flipping_over_problem.gif)

#### Possible solutions:

- Implement a spring-like script to control the suspension joint. That way, the joint could have a really low value of friction, but the script can put the joint in their extended state after compressing (to go over some obstacles, for example).

![](/imgs/suspension_real.gif)

### Not realistic enough sand - Unsolved

#### Problem nº 1:

As mentioned in the development section of this README, the sand doesn't have the mechanical features of a real life sand. They are colliding with the wheels and with one another, but it shouldn't make the wheel look like it is going to a small hill.

![](/imgs/sand.gif)

#### Possible solutions:

Implement friction and drag with proportional values in the .world file. Also, the friction of ground that the "population" is spawned should have friction values too.

#### Problem nº 2:

This process consumes a lot of memory and has a huge impact on performance. In the example shown, a population of 200 particles already causes a decrease in performance and in the Real time factor of the Gazebo.

### Visualization problems with LiDAR on RViz - Unsolved

#### Problem:

Although LiDAR normally publishes its values in its topic, RViz is unable to visualize the PointCloud generated by the plugin.

![](/imgs/lidar_rviz.png)

#### Possible solutions:

Create a new namespace for the plugin, since the plugin didn't adquire the group namespace of the gazebo, like the rest of the plugins.
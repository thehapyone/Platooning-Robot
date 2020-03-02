# Platooning-Robot [![MIT License](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/thehapyone/Platooning-Robot/blob/master/LICENSE)
This project is all about the quest of building a platooning capable robot. Aside being a platooning robot, it is capable of other things.

The repository contains codes that can be used for some of the following applications:
* Robot Platooning:
	* Side Formation
	* Follow the Leader
	* Change of Leader
* Lane Detection and Tracking
	* Lane Detection using Infrared Sensor
	* Lane Detection using the Robot Camera
* PID based Object Following
* Robot imitiating an Insect (Cyborg Mode)
* Robot Teleoperation

<img src="https://github.com/thehapyone/Platooning-Robot/blob/master/Robot/Assets/robot-image.jpeg" width="480">

## Table of Contents
* Project Objective
* Repository Structure
* Hardware Overview & Setup
* Software Overview & Setup
* Plato0ning in Robots
* Interesting Projects
* Code Snippets
* Project Usage
* Summary
* References

## Project Objective
The motivation for this repository and project came from one of my Master's degree course (Design of Embedded and Intelligent Systems). As part of the Design of Embedded and Intelligent Systems Course, it was required that students should
work together in building robots that can explore autonomously in an unknown environment and with the potential of integrating with biological organisms to act as a driver.

In the course “Design of Embedded and Intelligent Systems,” each year, small robots are built based on a theme, and robots exhibit that they can work in a coordinated environment. The theme for this year is
terraforming [1], which refers to modifying the environmental and topological structure of some unknown planet so that humans can have a habitat over that unknown planet while simultaneously
working with other like-minded robots.

<img src="https://github.com/thehapyone/Platooning-Robot/blob/master/Robot/Assets/stick_insect.png" width="100%">
Figure 1: A Large Stick Insect

Terraforming can solve some environmental problems and help us discover new elements and minerals that might be useful for humanity's survival on earth. The robot built as part of this project had different
modes, and one among them was cyborg mode in which stick insect controlled the robot’s movement.

As terraforming, platooning is also very important, which was another aspect of this project that was carried out. Platooning ensures robots can work together, have a leader to lead them or reassign a
leader in case of emergency, or even work in specific formations to achieve a goal.

## Repository Structure
This repository contains all the codes and resources used during the course of this project. They have been divided into groups. The **Robot** folder contains all the codes used so far. It contains the **Arduino**, **Assets** and **Main** sub-directory.
* Arduino Directory: This diretory contains all the arduino related codes used for this project in a version system. The Arduino codes only runs on the Sparkfun Redbot mainboard only.
* Assets: These contains assets related to this project.
* Main: This directory houses all of the code running on the Jetson Nano.
In the Main sub-directory, there is the **Extras**, **Final**, **Lane Detection**, **LCM**, and **ROS** directory. The **Extras** contains codes used mostly for configuration purposes, for example getting the key mappings of the used Gamepad controller.
The **Final** directory contains the main program running on the Jetson Nano/Raspberry Pi. It based a on a version system. The **Lane Detection** contains all the related codes used for lane detection and recognition on the robot.
**LCM** directory houses all the codes used for [LCM](https://lcm-proj.github.io/)[2] based communication. LCM known as Lightweight Communications and Marshalling was implemented in this project to allow different modules to communicate with easy other. The project was built a on modular independent architecture.
Lastly, we have the **ROS** directory. This directory holds all the ROS (Robot Operating System) codes and packages implemented during the course of the project.

## Hardware Setup
The robot used for this project as shown above is made up of the following:
* A Jetson Nano / Raspberry Pi 3
* SparkFun Inventors Kit for RedBot (Contains the Redbot Mainboard)
* Ultrasonic Sensor
* Micro Servo
* 3D Printed parts
* Accessories used with the Jetson:
	* Dual Mode Wirless NIC AC8265: 2.4GHz/5GHz Dual Mode Wifi and Bluetooth 4.2
	* 8MP CSI Camera 160 degree FOV
	* DC FAN: Mounted on the Nano

## Software Setup
For this project, a couple of software platform were used. The below are some of the high level software platform used:
* ROS Melodic
* Python 3 (For Main Programs)
* Python 2 (For ROS)
* TensorRT (For Deep Learning Optimization on the Jetson Nano
* Opencv (Image processing related)
* LCM (For Inter-communication between code modules in the system)
* Nvidia Digits (Training of custom model)
* and others.


 
## Interesting Projects
* A Jetson Nano Robot performing lane detection and recognition. Full video is available [here](https://youtu.be/PYo42D_4UOw)
	* <img src="/Robot/Assets/lane-following.gif" width="480">
* A Jetson Nano Robot and a Raspberry performing a side formation while working in platooning mode. Full video is available [here](https://youtu.be/_TgRxJMxzh8)
	* <img src="/Robot/Assets/side-formation.gif" width="480">
* A Jetson Nano Robot doing real-time detection of a stick insect and mimicking the insect movement. Full video is available [here](https://youtu.be/b76iaYdQWEo)
	* <img src="/Robot/Assets/insect-following.gif" width="480">


## References
[1] M. J. Fogg, "Terraforming Mars: A review of current research," Science and Technology Series, vol. 22, no. 3, pp. 415-420, 14 January 1998.
[2] LCM - Lightweight Communications and Marshalling, https://lcm-proj.github.io/




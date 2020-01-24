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

## Interesting Projects
* A Jetson Nano Robot performing lane detection and recognition
![](https://github.com/thehapyone/Platooning-Robot/blob/master/Robot/Assets/lane-following.gif)


## Repository Structure
The **Robot** folder contains all the codes used so far. It contains the **Arduino**, **Assets** and **Main** sub-directory.
* Arduino Directory: This diretory contains all the arduino related codes used for this project in a version system. The Arduino codes only runs on the sparfun redbot mainboard only.
* Assets: These contains assets related to this project.
* Main: This diretory houses all of the code running on the Jetson Nano.


<img src="https://github.com/thehapyone/Platooning-Robot/blob/master/Robot/Assets/robot-image.jpeg" width="480">

## Hardware Setup
The robot used for this project as shown above is made up of the following:
* A Jetson Nano
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






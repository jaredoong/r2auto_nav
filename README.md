# ROS2 foxy code for EG2310

## Mission
For this project, our turtlebot was required to complete the following tasks.

  1. Map out the entire maze
  2. Stop at a loading zone, identified by presence of NFC tag
  3. Move off after the ping pong balls have been loaded
  4. Find a heated object (tin can)
  5. Fire the balls at the heated object

The turtlebot 3 burger used was augmented with our customized launcher to fire off the balls at the target. For the navigation portion, a left wall-follower algorithm used utilised. Additional sensors used to complete the tasks include PN532 (NFC Reader) and AMG8833 (Thermal Camera). A push button was used to allow the turtlebot to move off when loaded.

## What's in this repo
This repository contains the code used for the mission in EG2310. The directory tree on the remote laptop controlling the turtlebot and the RPi on the turtlebot is also included below for reference. For this project, the RPi version used was RPi 3B+. The software, hardware, and steps required to replicate the working turtlebot is explained in detail in the documentation. 

## Directory tree on remote laptop
```bash
colcon_ws
├── build
├── install
├── log
└── src
    ├── auto_nav
    │   ├── resources
    │   ├── test
    │   ├── package.xml
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── auto_nav
    │       ├── __init__.py
    │       ├── package.xml
    │       ├── setup.py
    │       └── r2wall_follower.py
    └── custom_msgs
        ├── include
        ├── src
        ├── msg
        │   ├── Button.msg
        │   ├── Flywheel.msg
        │   ├── Launcher.msg
        │   ├── Nfc.msg
        │   └── Thermal.msg
        ├── CMakeLists.txt
        └── package.xml
```

## Directory tree on RPi
To be added later on

## Important files
* [r2wall_follower.py](https://github.com/jaredoong/r2auto_nav/blob/main/r2wall_follower.py) - This file contains the overall algorithm used for the mission. The main function used to navigate around the maze using the left wall-following algorithm is in ```left_follow_wall()```. The important functions used to complete the tasks include ```find_nfc()```, ```load_balls()```, ```find_thermal()```, ```launcher()```.
* [custom_msgs/msg] - This folder includes all the custom messages that are published by the nodes running on the remote laptop and the RPi in order to transmit important information. This package needs to exist on both the remote laptop and the RPi for the program to run properly. Steps for proper setup is described in detail [below](#Setting-up-software-on-remote-laptop).

Currently missing amg8833 library, pn532 library, sensors.py

## Important calibration required before starting
* ```threshold_temp``` - This is the temperature of the heated tin can in the mission. For greater accuracy when targeting the tin can, this should be set to slightly below the temperature of the tin can. It is important to ensure that this value is not set too close to the average temperature of the surroundings to prevent false triggering of the launcher
* ```TOTAL_NFC``` - This is the total number of detectable NFC in the maze. Due to the speed of the turtlebot, there is a possibility that the turtlebot is unable to sense the NFC tag even when it travels over it. Hence, it is necessary to test each NFC zone in the maze before the start of the mission. This is to ensure that the turtlebot stops only when it has completed the mapping of the maze. This constant value has to be updated in both [r2wall_follower.py](https://github.com/jaredoong/r2auto_nav/blob/main/r2wall_follower.py) and sensors.py.

## Setting up software on remote laptop


## Setting up software on the RPi

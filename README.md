# ROS2 code for EG2310

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
    │   ├── package.xml
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── auto_nav
    │       ├── __init__.py
    │       └── r2wall_follower.py
    └── custom_msgs
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
```bash
turtlebot_ws
├── build
├── install
├── log
└── src
    ├── sensors
    │   ├── package.xml
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── sensors
    │       ├── pn532
    │       ├── __init__.py
    │       └── sensors.py
    └── custom_msgs
        ├── msg
        │   ├── Button.msg
        │   ├── Flywheel.msg
        │   ├── Launcher.msg
        │   ├── Nfc.msg
        │   └── Thermal.msg
        ├── CMakeLists.txt
        └── package.xml
        
test_sensors
├── pn532
├── amg8833_i2c.py
├── test_button.py
├── test_NFC.py
├── test_servo.py
└── test_thermalcam.py
```

## Important files
* [r2wall_follower.py](https://github.com/jaredoong/r2auto_nav/blob/main/r2wall_follower.py) - This file contains the overall algorithm used for the mission. It contains subscribers that listen for the data received by the sensors, over topics in ```custom_msgs/msg```. The main function used to navigate around the maze using the left wall-following algorithm is in ```left_follow_wall()```. The important functions used to complete the tasks include ```find_nfc()```, ```load_balls()```, ```find_thermal()```, ```launcher()```.
* [custom_msgs/msg](https://github.com/jaredoong/r2auto_nav/tree/main/custom_msgs/msg) - This folder includes all the custom messages that are published by the nodes running on the remote laptop and the RPi in order to transmit important information. This package needs to exist on both the remote laptop and the RPi for the program to run properly. Steps for proper setup is described in detail [below](#Setting-up-software-on-remote-laptop).
* [sensors.py](https://github.com/jaredoong/r2auto_nav/blob/main/RPi_files/sensors/sensors/sensors.py) - This file contains the code to be run on the RPi. It is in charge of publishing the data from the sensors out to the remote laptop to let the remote laptop know when one task has been completed, which enables it to give the turtlebot the command to move on to the next task.
* [pnc532](https://github.com/jaredoong/r2auto_nav/tree/main/RPi_files/sensors/sensors/pn532) - This folder contains the library needed to use the PN532 NFC reader in I<sup>2</sup>C mode. For ```sensors.py``` to run properly, this folder needs to be in the same directory as ```sensors.py```.
* [test_sensors](https://github.com/jaredoong/r2auto_nav/tree/main/test_sensors) - This folder contains the python files that can be used to check if the hardware is functional. Included are files to test the button, servo, NFC reader,and thermal camera.

## Important libraries to download onto the RPi
### On the RPi
* adafruit_amg88xx library - This library is required for ```sensors.py``` to be able to read in the data from the AMG8833 Thermal camera. To install the library, run the following code in the terminal.
> sudo pip3 install adafruit-circuitpython-amg88xx
* pigpio - This library is used to prevent servo jitter. To install on the RPi, follow the instructions [here](https://abyz.me.uk/rpi/pigpio/download.html). Before using the pigpio library, it is necessary to start the pigpio daemon when the RPi is booted up. Run the following code in the terminal before running ```sensors.py```.
> sudo pigpiod

## Useful tools for the RPi
* i2c-tools - This is to help check if the I<sup>2</sup>C devices are connected and detectable by the RPi. To install, run the following command in the terminal.
> sudo apt-get install -y i2c-tools
* To check if the I<sup>2</sup>C devices can be detected by the RPi, run the following command in the terminal.
> sudo i2cdetect -y 1

## Important calibration required before starting
* ```threshold_temp``` - This is the temperature of the heated tin can in the mission. For greater accuracy when targeting the tin can, this should be set to slightly below the temperature of the tin can. It is important to ensure that this value is not set too close to the average temperature of the surroundings to prevent false triggering of the launcher
* ```TOTAL_NFC``` - This is the total number of detectable NFC in the maze. Due to the speed of the turtlebot, there is a possibility that the turtlebot is unable to sense the NFC tag even when it travels over it. Hence, it is necessary to test each NFC zone in the maze before the start of the mission. This is to ensure that the turtlebot stops only when it has completed the mapping of the maze. This constant value has to be updated in both [r2wall_follower.py](https://github.com/jaredoong/r2auto_nav/blob/main/r2wall_follower.py) and [sensors.py](https://github.com/jaredoong/r2auto_nav/blob/main/RPi_files/sensors/sensors/sensors.py).

## Setting up software on remote laptop
1. Ensure that you have Ubuntu 20.04 and ROS 2 Foxy on your laptop. Refer [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) on how to install the required software. Ensure that you are following the instruction under the "Foxy" tab.
2. 

## Setting up software on the RPi
1. Using Ubuntu, follow the instructions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup) burn the ROS 2 Foxy Image to the SD card onto the RPi on the turtlebot. Follow through the "Quick Start Guide" got a working turtlebot.
2. Create a 

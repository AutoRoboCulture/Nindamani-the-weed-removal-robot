# Nindamani-the-weed-removal-robot

Nindamani, the AI based mechanically weed removal robot, which autonomously detects and segment the weeds from crop using artificial intelligence. The whole robot modules natively build on ROS2. Nindamani can be used in any early stage of crops for autonomous weeding.

In this following repository, you will find instructions for software installation and control mechanism for Nindamani robot.

# Features:
  - Fully ROS2 compatible
  - Battery Operated
  - Runtime upto 8-10 hours
  - Robotics Arm based weed removal
  - Weed detection accuracy upto 85%
  - Easy to Operate
 
# Software Specifications:

| Parameter | Value |
| ------------- | ------------- |
| Robotics OS | ROS2.0 Dashing Diademata |
| System | Ubuntu 18.04 LTS |
| Communication | Intel 8265 Wifi card |
| AI Framework | Keras |
| Programming Language | Python3 & C |


# Hardware Specifications:

| Parameter | Value |
| ------------- | ------------- |
| Degrees of freedom | 3 DOF |
| Error  | ±2 mm |
| Payload | 1.5 kg |
| Weight | 35 kg |
| Height | 740 to 860 mm |
| Width | 980 mm |
| Arm Reach | 200x200 sq mm |
| Processor board | Jetson nano Dev Kit |
| Microcontroller | Arduino Mega |
| Motors | Servo and Steppers |
| Camera | RPi cam ver.2 |
| Battery | 48V 30ah |


# Installation on Jetson Nano Dev Kit

## 1. NVIDIA Jetpack SDK
  - Download latest SDK image: https://developer.nvidia.com/embedded/jetpack
  - Completely Format SD card (should not contain any partition). Use Ubuntu default app **Disks** [Recommeded 64GB SD card]
  - Copy ZIP(jetpack image) file to SD card: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write
## 2. Prerequisites and Dependencies for TensorFlow
  - Install Keras: `sudo pip3 install keras`
  - Follow this instructions [official from NVIDIA]: https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html#prereqs
## 3. ROS2 (Dashing Diademata)
  - Install ROS2 base: https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/
  - Make sure that you have colcon in your machine if you are installing from Debian packages. `sudo apt install python3-colcon-common-extensions`
  - For adding additional packages use: `sudo apt install ros-$CHOOSE_ROS_DISTRO-<package-name>`
  
## 4.Arduino
  - Download: https://www.arduino.cc/en/guide/linux
  - To get Temporary access to USB: `sudo chown <user-name> /dev/tty<usb>` and `sudo chmod a+rw /dev/tty<usb>`
  - To set Permenantly change USB device permission: http://ask.xmodulo.com/change-usb-device-permission-linux.html
  - To control arduino from Command line Source:https://github.com/arduino/arduino-cli

## 5. OpenCV 3.4.4
  - Refer link: https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
  
## 6. Wifi
  - To setup default wifi connection(Intel 8265 NGW card) while bootup Source: https://desertbot.io/blog/how-to-add-a-dual-wifi-bluetooth-card-to-a-jetson-nano-intel-8265

# Download preTrained Model weights 
  Link for MASK-RCNN [preTrained model](https://drive.google.com/open?id=1bXEOmOsoBLpXQWVvhhJvhD-RjvCLxOAQ)



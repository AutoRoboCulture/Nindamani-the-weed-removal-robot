# RPICAM AI Interface package contains following files
This package is used for controlling the rpi camera with AI interface
  
  1.  `scripts/rpicam_ai_interface.py`
  2.  `scripts/perspectiveImage.py`
  3.  `scripts/camera.py`
  4.  `CMakeLists.txt`
  5.  `package.xml`

# Description:
1. **scripts/rpicam_ai_interface.py**: Open up the rpi camera and load pretrained MASK RCNN model for predicting the weeds and crop. And publish the detected pixel cordinates to ROS2 topic`.

   **Inputs**: Subscribes to *takePicAgain* topic and publish data on *objectcoordinates*.

2. **scripts/perspectiveImage.py**: It is used for converting the angle viewed image to an bird-eye view image
   **Inputs**: Takes the input image from rpicam camera
   
3. **scripts/camera.py**: NVIDIA specific camera driver for loading up the camera

4. **CMakeLists.txt**: List down all necessary dependant packages and other directories/files of rpicam_ai_interface package.

5. **package.xml**: Includes dependent build in this file.

Note: To get better understanding on CMakeLists.txt and package.xml files refer [cmakeliststxt-vs-packagexml](https://answers.ros.org/question/217475/cmakeliststxt-vs-packagexml/?answer=217488#post-id-217488).

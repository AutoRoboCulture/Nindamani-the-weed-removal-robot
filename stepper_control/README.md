# stepper_control package contains following files.
This package is used for controlling the multiple stepper motors with ROS2 interface

  1.  `config/stepper_settings.yaml`
  2.  `scripts/stepper_driver.py`
  3.  `scripts/stepper_ros2_interface.py`
  4.  `src/deltaKinematics.c`
  5.  `srv`
  3.  `CMakeLists.txt`
  4.  `package.xml`

# Description:
1. **config/stepper_settings.yaml**: This is ROS2 parameter file. It will be used for setting up the default value at initial. 
  
2. **scripts/stepper_driver.py**: It decodes the upper level serial command and arrange it in the UART format to send on USB-UART device.

3. **scripts/stepper_ros2_interface.py**: It takes the weed coordinates and move the steppers according to the given location.
   
   **Inputs**: Subscribes to *weedCoordinates* topic and publish data to Atmega2560 controller.

4. **src/deltaKinematics.c**: Its an inverse delta kinematic library for getting the exact theta's for all stepper motor from the input (x,y,z) location

5. **srv**: It contains all the services specifically related to stepper motor

6. **CMakeLists.txt**: List down all necessary dependant packages and other directories/files of stepper_control package.

7. **package.xml**: Includes dependent build in this file.

Note: To get better understanding on CMakeLists.txt and package.xml files refer [cmakeliststxt-vs-packagexml](https://answers.ros.org/question/217475/cmakeliststxt-vs-packagexml/?answer=217488#post-id-217488).

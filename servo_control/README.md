# Package Description: 
  This is a ROS2 package for DC Servo Motor. Please refer the [datasheet](https://github.com/AutoRoboCulture/nindamani-the-weed-removal-robot/blob/master/servo_control/RMCS220x_DCServo%2BDriver.pdf) of the motor which is included in the package before proceeding further. This package uses serial communication to control the motor. It requires a USB-to-UART converter to interface the motor with a computer.

# servo_control package contains 6 files.

  1.  config/servo_settings.yaml
  2.  scripts/servo_driver.py 
  3.  scripts/servo_ros2_interface.py
  4.  scripts/__init__.py
  5.  CMakeLists.txt
  6.  package.xml

# Description:
1. **config/servo_settings.yaml**: This is configuration file for servo motor communication. 
  
   **Inputs**: It takes servo motor serial port name and motor rpm speed (0-255)

2. **scripts/servo_driver.py**: It decodes upper level serial commands and arrange it in UART format to send on USB-UART.

   **Inputs**: Inputs from servo_ros2_interface.py and send it over to USB-UART communication cable.

3. **scripts/scripts/servo_ros2_interface.py**: It consists servo motor control services and topics. Main used for Open and CLose Gripper actions.

   **Inputs**: Acts as client for other packages.

4. **scripts/__init__.py**: Making files into python modules.

5. **CMakeLists.txt**: List down all necessary dependant packages and other directories/files of nindamani_agri_robot package.

6. **package.xml**: Includes dependent build in this file.

Note: To get better understanding on CMakeLists.txt and package.xml files refer [cmakeliststxt-vs-packagexml](https://answers.ros.org/question/217475/cmakeliststxt-vs-packagexml/?answer=217488#post-id-217488).

# nindamani_agri_robot package contains 4 files.
  1.  launch/nindamani_agri_robot.launch.py
  2.  scripts/nindamani_agri_robot.py
  3.  CMakeLists.txt
  4.  package.xml

# Description:
1. **launch/nindamani_agri_robot.launch.py**: This is ROS2 launch file. It will launch/start nodes of each package. 
  
   **Inputs**: It takes three inputs for each package listed. 1) Package name, 2) Executable node file(.py file in our case), 3) Parameters(if applicable)

2. **scripts/nindamani_agri_robot.py**: This file takes weed coordinates and perform pick and place sequence on each weed. Including gripper Open and Close actions.

   **Inputs**: Subscribes to *objectcoordinates* topic and publish data on *weedCoordinates*.

3. **CMakeLists.txt**: List down all necessary dependant packages and other directories/files of nindamani_agri_robot package.

4. **package.xml**: Includes dependent build in this file.

Note: To get better understanding on CMakeLists.txt and package.xml files refer [cmakeliststxt-vs-packagexml](https://answers.ros.org/question/217475/cmakeliststxt-vs-packagexml/?answer=217488#post-id-217488).

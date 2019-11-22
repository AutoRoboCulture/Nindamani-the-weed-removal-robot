# RPICAM AI Interface
Following codes are used for controlling the rpi camera with AI interface

# rpicam_camera_interface file
Open up the rpi camera and load pretrained MASK RCNN model for predicting the weeds and crop. And publish the detected pixel cordinates to ROS2 topic 

# Main Functions and description:
1) def initialize_capture_queue()
- Input: `publish y/n for taking pics or not on topic 'takePicAgain' topic`
- Output: `None`
- Description: `It initialize the camera for taking pictures`

2) def weedLocation()
- Input: `take the clicked image as input`
- Output: `returns the weed location and publish it to the 'ObjectCordinates' topic`
- Description: `It takes the image and predict the weed from crop, publish the weed location to ROS2 topic`

3) def manualWeedLocation()
- Input: `take the test location of weeds like for ex: (G100A50B-432C) 100A-> x cord, 50B-> y cord, -432C-> z cord by publishing it to topic 'manualWeedLoc'`
- Output: `None`
- Description: `It is for testing the overall mechanism of weed removal robot by providing manual weed coordinates`

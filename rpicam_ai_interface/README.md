# RPICAM AI Interface
Following codes are used for controlling the rpi camera with AI interface

# rpicam_camera_interface file
Open up the rpi camera and load pretrained MASK RCNN model for predicting the weeds and crop. And publish the detected pixel cordinates to ROS2 topic 

# Main Functions and descriprion:
1) def initialize_capture_queue()
- Input: `<publish y/n for taking pics or not on topic 'takePicAgain' topic`
- Ouput: `None`

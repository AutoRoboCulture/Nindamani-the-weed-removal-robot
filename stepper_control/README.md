# rhino_dc_servo
* This is a ROS package for Rhino DC Servo Motor. Please refer the datasheet of the motor which is included in the package before proceeding further. This package uses serial communication to control the motor. It requires a USB-to-UART converter to interface the motor with a computer.

## Procedure:
* `sudo apt install ros-kinetic-swri-rospy`
* Set the `port` and `rpm` of the motor in `config/settings.yaml` file.
* Make sure `roscore` is running.
* `roslaunch rhino_dc_servo rhino.launch`
* You should give access to the port before executing roslaunch. For example,
```
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyUSB0
```
* You can give command to the motor on the following topics, `motor_speed`, `absolute_cmd` and `relative_cmd` with units `rad/s`, `rad` and `rad` respectively.
* The encoder data is published over the topic `encoderTicks`.
* The other parameters can be set through ROS Services.
* If you are using motors for a differential drive robot, you can use the launch file `rhino_differential.launch`.
* `https://youtu.be/0Q5lQMK9zoE` - a differential drive robot(fitted with Rhino DC Servo motors) doing autonomous navigation inside my home.

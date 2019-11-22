#!/usr/bin/python3

import pdb
import rclpy
import time
from stepper_driver import StepperMotor
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16, String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from rhino_servo.srv import *
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.impl.rcutils_logger import RcutilsLogger as log
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity
from ctypes import *

#Uploading arduino code using python file (Note: "String" line is basically terminal code)
import os
os.system("arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega ../Arduino/stepperMotorControl/stepperMotorControl/")
time.sleep(5)

MICROSTEP = 6400                    #Aligned with Stepper Driver, Switches set on Stepper Driver

class Stepper:
    #constructor
    def __init__(self):
        super().__init__()
        self.res = False 
        self.initUI()

    def initUI(self):

        rclpy.init(args=None)
        self.node2 = rclpy.create_node('stepperMotor', automatically_declare_parameters_from_overrides=True)
        
        #Initiallizing the Stepper Motor Driver
        self.myMotor = StepperMotor(self.node2.get_parameter_or(self.node2.get_namespace() + 'port', '/dev/ttyUSB0'))
        
        #Checking if Stepper motor port is availaible or not 
        self.myMotor.openSerialPort()
        self.myMotor.isAvailable()

        self.motorMicrostep = MICROSTEP
        
        #Initiallizing & Loading Delta Kinematics C library
        self.libDeltaKinematics = CDLL("install/stepper_control/lib/libdeltaKinematics.so") 
        
        class ReVal(Structure):
            _fields_ = [("theta1", c_float),("theta2", c_float),("theta3", c_float)]

        self.libDeltaKinematics.delta_calcInverse.argtypes = [c_float, c_float, c_float]
        self.libDeltaKinematics.delta_calcInverse.restype = ReVal
        
        #Subscribers
        self.node2.create_subscription(Float64,'stepper_motor_speed', self.setSpeed)
        self.node2.create_subscription(Float64,'stepperMotor1_absolute_cmd',self.setAbsoluteMotor1)
        self.node2.create_subscription(Float64,'stepperMotor2_absolute_cmd',self.setAbsoluteMotor2)
        self.node2.create_subscription(Float64,'stepperMotor3_absolute_cmd',self.setAbsoluteMotor3)
        self.node2.create_subscription(String,'control_stepperMotors_cmd',self.setSteppersControl)
        self.node2.create_subscription(Float64,'set_all_stepper_microstep',self.setMotorMicrostep)
        self.node2.create_subscription(Int32,'test_triangleMotion',self.testTriangleMotion)
        self.node2.create_subscription(String,'testObjectcoordinates', self.testMotorControl)
        self.node2.create_subscription(String,'weedCoordinates', self.MotorControlFeedback)
        
        #Publisher
        self.ack_publisher = self.node2.create_publisher(String,'controlFeedback', 10)

        #Services
        self.set_motor_acceleration = self.node2.create_service(SetMotorAcceleration, 'stepper_set_motor_acceleration', self.setMotAcc)
        self.get_motor_acceleration = self.node2.create_service(GetMotorAcceleration, 'stepper_get_motor_acceleration', self.getMotAcc)
        self.get_motor_microstepping = self.node2.create_service(GetMotorMicrostepping, 'stepper_get_motor_microstep', self.getMotMicrostep)
        self.auto_calibrate = self.node2.create_service(AutoCalibrate, 'auto_calibrate', self.autoC)
        
        rclpy.spin(self.node2)

    
    def testMotorControl(self,msg):
       
        #Decoding the message of format "G<x>A<y>B<z>C"
        cordData = msg.data
        cordA = cordData.find("A")
        cordB = cordData.find("B")
        position1 = float(cordData[1:cordA])
        position2 = float(cordData[cordA+1:cordB])
        position3 = float(cordData[cordB+1:-1])

        #Setting Offset for delta library to reach object location with more accurate
        if (position1>0):
            position1 = position1*1.1550
        
        if (position1<0):
            position1 = position1*1.1642           #1.1642 factor getting from average error by calibrating x&y axis
        
        if (position2>0):
            position2 = position2*1.1226
        
        if (position2<0):
            position2 = position2*1.1363

        
        if((position1 <=120 and position1>=-120)and(position2 <=120 and position2>=-120)and(position3 <=-270 and position3>=-520)):
            res = self.libDeltaKinematics.delta_calcInverse(position1,position2,position3)
            theta1 = res.theta1
            theta2 = res.theta2
            theta3 = res.theta3
        
            #The delta library calculate end pose when three arm is parallel to ground
            theta1 = (-65) - theta1         #65 degree means when z is -411mm( arm is parallel to ground)
            theta2 = (-65) - theta2
            theta3 = (-65) - theta3

            #Conversion factor from degree to microstep for stepper motor
            pos1 = round(theta1*17.77778)     #(self.motorMicrostep/360))
            pos2 = round(theta2*17.77778)     #(self.motorMicrostep/360))
            pos3 = round(theta3*17.77778)     #(self.motorMicrostep/360))

            #Controlling the stepper motor
            self.myMotor.setAbsolutePosition(pos1,pos2,pos3)
            time.sleep(0.3)

        else:
            self.node2.get_logger().info('Value Out of Range!!')

    def MotorControlFeedback(self,msg):
        
        #Decoding the message of format "G<x>A<y>B<z>C"
        cordData = msg.data
        cordA = cordData.find("A")
        cordB = cordData.find("B")
        position1 = float(cordData[1:cordA])
        position2 = float(cordData[cordA+1:cordB])
        position3 = float(cordData[cordB+1:-1])
        
        #Setting Offset for delta library to reach object location with more accurate
        if (position1>0):
            #position1 = position1*1.182           #1.182 factor getting from average error by calibrating x&y axis
            position1 = position1*1.1550
        
        if (position1<0):
            position1 = position1*1.1642           #1.182 factor getting from average error by calibrating x&y axis
        
        if (position2>0):
            #position2 = position2*1.182
            position2 = position2*1.1226
        
        if (position2<0):
            #position2 = position2*1.182
            position2 = position2*1.1363

        
        if((position1 <=120 and position1>=-120)and(position2 <=120 and position2>=-120)and(position3 <=-270 and position3>=-520)):
            res = self.libDeltaKinematics.delta_calcInverse(position1,position2,position3)
            theta1 = res.theta1
            theta2 = res.theta2
            theta3 = res.theta3
        
            #The delta library calculate end pose when three arm is parallel to ground
            theta1 = (-65) - theta1         #65 degree means when z is -411mm( arm is parallel to ground)
            theta2 = (-65) - theta2
            theta3 = (-65) - theta3

            #Conversion factor from degree to microstep for stepper motor
            pos1 = round(theta1*17.77778)    #(self.motorMicrostep/360))
            pos2 = round(theta2*17.77778)    #(self.motorMicrostep/360))
            pos3 = round(theta3*17.77778)    #(self.motorMicrostep/360))

            #Controlling the stepper motor
            self.myMotor.setAbsolutePosition(pos1,pos2,pos3)
        else:
            self.node2.get_logger().info('Value Out of Range!!')
   
    def setSpeed(self, msg):
        speed = (msg.data * self.motorMicrostep)/60   #frequency = ((speed x 60)/microstep) 
        self.myMotor.setMotorSpeed(speed)
    
    def setSteppersControl(self, msg):
        degData = msg.data
        degA = degData.find("A")
        degB = degData.find("B")
        degree1 = float(degData[1:degA])
        degree2 = float(degData[degA+1:degB])
        degree3 = float(degData[degB+1:-1])
        
        if((degree1 <=0 and degree1>=-128)and(degree2 <=0 and degree2>=-128)and(degree3 <=0 and degree3>=-128)):
            pos1 = float(degData[1:degA]) * (self.motorMicrostep/360)
            pos2 = float(degData[degA+1:degB]) * (self.motorMicrostep/360)
            pos3 = float(degData[degB+1:-1]) * (self.motorMicrostep/360)
            self.myMotor.setAbsolutePosition(round(pos1), round(pos2), round(pos3))
        else:
            self.node2.get_logger().info('Value Out of Range!!')

    def testTriangleMotion(self,msg):               #For testing the Triangle motion of the delta robot
        self.myMotor.testTriangle(msg.data)         #msg format ("T<val>": <val>no. of times the motion repeated)
      

      
    #Controlling the Individual Stepper Motor
    def setAbsoluteMotor1(self, msg):
        position1 = msg.data
        if(position1 <=0 and position1>=-128):
            position1 = msg.data * (self.motorMicrostep/360)
            self.myMotor.setAbsolutePosition(round(position1), 0, 0)
        else:
            self.node2.get_logger().info('Value Out of Range!!')

    def setAbsoluteMotor2(self, msg):
        position2 = msg.data
        if(position2 <=0 and position2>=-128):
            position2 = msg.data * (self.motorMicrostep/360)
            self.myMotor.setAbsolutePosition(0, round(position2), 0)
        else:
            self.node2.get_logger().info('Value Out of Range!!')

    def setAbsoluteMotor3(self, msg):
        position3 = msg.data
        if(position3 <=0 and position3>=-128):
            position3 = msg.data * (self.motorMicrostep/360)
            self.myMotor.setAbsolutePosition(0, 0, round(position3))
        else:
            self.node2.get_logger().info('Value Out of Range!!')
    
    def setMotorMicrostep(self, msg):
        MICROSTEP = msg.data
    
    
    ##Service Callback Function

    def getMotMicrostep(self, request, response): 
        response.microstep = self.myMotor.getMotorMicrostep()
        return response
    
    
    def getMotAcc(self, request, response):
        response.acceleration = (self.myMotor.getMotorAcceleration())
        return response 
    
    def setMotAcc(self, request, response):
        response.success = (self.myMotor.setMotorAcceleration(request.acceleration))
        return response 
    

    def autoC(self, request, response):
        response.success = self.myMotor.autoCalibrate()
        return response


if __name__ == '__main__':
    a = Stepper()
   

#!/usr/bin/python3

import pdb
import rclpy
import time

#Import servo driver 
from servo_driver import ServoMotor
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16, String
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from servo_control.srv import *
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.impl.rcutils_logger import RcutilsLogger as log
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity
from ctypes import *

class Servo:
    #constructor
    def __init__(self):
        super().__init__()

        self.initUI()
    
    #Input: None 
    #Output: None
    #Description: create nodes for servo motor, create a ros publisher topic and three ros topic subscriber, 
    # setup services for taking control over servo motor
    def initUI(self):

        rclpy.init(args=None)

        self.node = rclpy.create_node('Servomotor', automatically_declare_parameters_from_overrides=True)
        
        #set USB port for servo motor
        self.myMotor = ServoMotor(self.node.get_parameter_or(self.node.get_namespace() + 'port', '/dev/ttyUSB1'))
        self.rpm = self.node.get_parameter_or(self.node.get_namespace() + 'rpm', 200)
        self.r = self.node.get_parameter_or(self.node.get_namespace() + 'rate', 5)

        #setting up encoder data publisher
        self.encoder_pub = self.node.create_publisher(Int32,'encoderTicks', 10)
        
        self.myMotor.openSerialPort()
        #Sending 'P0' by default
        self.myMotor.isAvailable()

        #setting up subscribers
        self.node.create_subscription(Float64,'servo_motor_speed', self.setSpeed) #Set speed 'S'
        self.node.create_subscription(Float64,'servo_absolute_cmd',self.setAbsolute) #Set Position 'G'
        self.node.create_subscription(Float64,'servo_relative_cmd',self.setRelative) #Set relative postion 'R'
        
        #setting up services
        self.read_damping = self.node.create_service(ReadDamping, 'servo_read_damping', self.dampingRead) # 'D'
        self.write_damping = self.node.create_service(WriteDamping, 'servo_write_damping', self.dampingWrite) # 'D'
        self.load_factory_settings = self.node.create_service(LoadFactorySettings, 'servo_load_factory_settings', self.loadFactory) # 'Y'
        self.set_position_encoder = self.node.create_service(SetPositionEncoder, 'servo_set_position_encoder', self.setPositionEnc) # 'P'
        self.get_absolute_position = self.node.create_service(GetAbsolutePosition, 'servo_get_absolute_position', self.getAbsPos) # 'G'
        self.set_feedback_gain = self.node.create_service(SetFeedbackGain, 'servo_set_feedback_gain', self.setFG) # 'A'
        self.get_feedback_gain = self.node.create_service(GetFeedbackGain, 'servo_get_feedback_gain', self.getFG) # 'A'
        self.set_proportionate_gain = self.node.create_service(SetProportionateGain, 'servo_set_proportionate_gain', self.setPG) # 'B'
        self.get_proportionate_gain = self.node.create_service(GetProportionateGain, 'servo_get_proportionate_gain', self.getPG) # 'B'
        self.get_integral_gain = self.node.create_service(GetIntegralGain, 'servo_get_integral_gain', self.getIG) # 'C'
        self.set_integral_gain = self.node.create_service(SetIntegralGain, 'servo_set_integral_gain', self.setIG) # 'C'
        self.set_max_motor_speed = self.node.create_service(SetMaxMotorSpeed, 'servo_set_max_motor_speed', self.setMaxSpeed) # 'M'
        self.get_max_motor_speed = self.node.create_service(GetMaxMotorSpeed, 'servo_get_max_motor_speed', self.getMaxSpeed) # 'M'
        self.auto_calibrate = self.node.create_service(AutoCalibrate, 'servo_auto_calibrate', self.autoC) # 'X'
        self.open_gripper = self.node.create_service(OpenGripper, 'servo_open_gripper', self.openGripper)
        self.close_gripper = self.node.create_service(CloseGripper, 'servo_close_gripper', self.closeGripper)
        
        rclpy.spin(self.node)

    def update(self):
        data = Int32()
        data.data = self.myMotor.getPositionEncoder()
        if data.data != None :
            self.encoder_pub.publish(data)

    def __del__(self):
        self.myMotor.closeSerialPort()
   
    def setSpeed(self, msg):
        speed = msg.data * 9.5492965855137 * 65000 /self.rpm /self.maxSpeed #255 # ????????
        self.myMotor.writeMotorSpeed(speed)

    def setAbsolute(self, msg):
        #position = msg.data * 286.4788976 #(for radian)
        position = msg.data #giving pulse count input direct 
        result = self.myMotor.setAbsolutePostion(position)
    
    def openGripper(self, request, response):
        response.success = self.myMotor.setAbsolutePostion(3000) #opening of jaw #Motor Full rotation value is 1800
        return response
    
    def closeGripper(self,request, response):
        response.success = self.myMotor.setAbsolutePostion(0) #closing of jaw
        return response

    def setRelative(self, msg):
        position = msg.data
        self.myMotor.setRelativePostion(position)

    def dampingRead(self, request, response):
        response.damp = self.myMotor.readDamping()
        return response
    
    def dampingWrite(self, request, response):
        response.success = self.myMotor.writeDamping(request.damp)
        return response

    def loadFactory(self, request, response):
        response.success = self.myMotor.loadFactorySettings()
        return response

    def setPositionEnc(self, request, response):
        response.success = self.myMotor.setPositionEncoder(request.encoder)
        return response

    def getAbsPos(self, request, response):
        response.position = (self.myMotor.getAbsolutePostion())
        return response 

    def setFG(self, request, response):
        response.success = self.myMotor.setFeedbackGain(request.gain)
        return response

    def getFG(self, request, response):
        response.gain =(self.myMotor.getFeedbackGain())
        return response 

    def setPG(self, request, response):
        response.success = self.myMotor.setProportionateGain(request.pgain)
        return response

    def getPG(self, request, response):
        response.pgain =  (self.myMotor.getProportionateGain())
        return response 

    def setIG(self, request, response):
        response.success = self.myMotor.setIntegralGain(request.igain)
        return response

    def getIG(self, request, response):
        response.igain = (self.myMotor.getIntegralGain())
        return response

    def setMaxSpeed(self, request, response):
        response.success = self.myMotor.setMaxMotorSpeed(request.mspeed)
        return response

    def getMaxSpeed(self, request, response):
        response.mspeed = self.myMotor.getMaxMotorSpeed()
        return response

    def autoC(self, request, response):
        response.success = self.myMotor.autoCalibrate()
        return response

    def setup(self):
        self.myMotor.writeDamping(0)
        self.myMotor.setMaxMotorSpeed(255)

if __name__ == '__main__':
    a = Servo()
   

#!/usr/bin/python3

#Import necessary library
import pdb
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
import numpy as np
import time
from ctypes import *

#Import open and close services from Servo motor
from servo_control.srv import OpenGripper,CloseGripper

class Nindamani:
    def __init__(self):
        super().__init__()
        self.res = False
        self.initUI()

    def initUI(self):
        rclpy.init(args=None)

        self.node1 = rclpy.create_node('nindamani',allow_undeclared_parameters=True)
        
        #Setting publisher (weedCoordinates publishes cords to stepper motor)
        self.cord_publisher = self.node1.create_publisher(String,'weedCoordinates', 10)

        #Setting up subscriber for objectCordinates
        #objectcoordinates subscribes to rpicam_AI_interface.py
        self.node1.create_subscription(Float32MultiArray,'objectcoordinates', self.autoPickNplace)

        #creating client for open and close gripper
        self.openGripperClient = self.node1.create_client(OpenGripper, 'servo_open_gripper')
        self.closeGripperClient = self.node1.create_client(CloseGripper, 'servo_close_gripper')

        rclpy.spin(self.node1)

    def autoPickNplace(self,msg):
        if (msg.data != None):
            
            #Extracting object count and its location coordinates
            objCnt = msg.layout.dim[0].size
            stride = msg.layout.dim[0].stride
            objLoc = msg.data
            
            #Looping for number of object detected on image
            for i in range(0,objCnt*3,stride):
                xyzCord = objLoc[i:i+stride]
                position1 = round(xyzCord[0])
                position2 = round(xyzCord[1])
                position3 = round(xyzCord[2])
                
                #Keeping limit on delta robot working region
                if((position1 <=100 and position1>=-100)and(position2 <=100 and position2>=-100)and(position3 <=-270 and position3>=-520)):

                    #above weed cord
                    pos3 = position3 + 65 
                    
                    #---SEQUENCE 1---#
                    #Publishing coordinates to steppermotor_ros2.py
                    msg = String()
                    msg.data = ('G'+(str(position1)+'A')+(str(position2)+'B')+(str(pos3)+'C'))
                    self.cord_publisher.publish(msg) 

                    time.sleep(1.5) #manual entered delay to complete action
            
                    #---SEQUENCE 2---#
                    #for opening gripper
                    self.req = OpenGripper.Request()
                    while not self.openGripperClient.wait_for_service(timeout_sec=1.0):
                        self.node1.get_logger().info("service not available, waiting again...")
                    future = self.openGripperClient.call_async(self.req)
                
                    time.sleep(1)

                    #---SEQUENCE 3---#
                    msg = String()
                    msg.data = ('G'+(str(position1)+'A')+(str(position2)+'B')+(str(position3)+'C'))
                    self.cord_publisher.publish(msg) 
    
                    time.sleep(2)
    
                    #---SEQUENCE 4---#
                    #for closing gripper
                    self.req = CloseGripper.Request()
                    while not self.closeGripperClient.wait_for_service(timeout_sec=1.0):
                        self.node1.get_logger().info("service not available, waiting again...")
                    future = self.closeGripperClient.call_async(self.req)
                
                    #---SEQUENCE 5---#
                    msg = String()
                    msg.data = ('G'+(str(position1)+'A')+(str(position2)+'B')+(str(pos3)+'C'))
                    self.cord_publisher.publish(msg) 

                    time.sleep(1.5)

                    #---SEQUENCE 6---#
                    #basket location, object drop location
                    msg.data = ('G'+(str(-100)+'A')+(str(-100)+'B')+(str(pos3)+'C'))
                    self.cord_publisher.publish(msg) 

                    time.sleep(2.5)

                    #---SEQUENCE 7---#
                    #for opening gripper
                    self.req = OpenGripper.Request()
                    while not self.openGripperClient.wait_for_service(timeout_sec=1.0):
                        self.node1.get_logger().info("service not available, waiting again...")
                    future = self.openGripperClient.call_async(self.req)
                
                    time.sleep(0.5)

            #---SEQUENCE 8---#
            #default position, task_completed/rest postion
            msg = String()
            msg.data = ('G'+(str(0)+'A')+(str(0)+'B')+(str(-300)+'C'))
            self.cord_publisher.publish(msg) 

            time.sleep(1.5)
        
            #---SEQUENCE 9---#
            #for closing gripper
            self.req = CloseGripper.Request()
            while not self.closeGripperClient.wait_for_service(timeout_sec=1.0):
                self.node1.get_logger().info("service not available, waiting again...")
            future = self.closeGripperClient.call_async(self.req)


if __name__ == '__main__':
    a = Nindamani()


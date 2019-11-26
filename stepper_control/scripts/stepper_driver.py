#!/usr/bin/python3
import serial
import rclpy
import time
import six
from rclpy.impl.rcutils_logger import RcutilsLogger as log
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity

MICROSTEP = 6400 #Set it to 6400 on stepper motor 6A driver using 7 small white switches

#A - Read/Write for Stepper Acceleration
#S - Read/Write for Stepper Speed
#G - Read/Write for Stepper Position
#X - Setting the default positon of delta robot
#T - Testing Triangle motion of delta robot

class StepperMotor:
    __port = ''
    __ser = serial.Serial(timeout=1, baudrate=115200)

    def __init__(self, serial_port_):
        StepperMotor.__port = serial_port_

    def openSerialPort(self):
        StepperMotor.__ser.port = StepperMotor.__port
        try:
            StepperMotor.__ser.open()
            rclpy.logging._root_logger.info('Opened the port'+ StepperMotor.__port)
        except:
            rclpy.logging._root_logger.fatal('Cannot open the port '+ StepperMotor.__port)
            rclpy.shutdown()

    def closeSerialPort(self):
        StepperMotor.__ser.close()

    def getMotorAcceleration(self):
        StepperMotor.__ser.write(('A\r').encode())
        self.d = StepperMotor.__ser.readline()
        rclpy.logging._root_logger.debug('A'+str((self.d)) + '\r')
        self.p = self.d.find(('A').encode())
        self.q = self.d.find((' ').encode())
        self.d = self.d[self.p + 1 :]
        self.d = float(self.d) - 65.0
        return self.d

    def setMotorAcceleration(self, accel):
        rclpy.logging._root_logger.debug('A'+str(float(accel)) + '\r')
        StepperMotor.__ser.reset_output_buffer()
        try:
            StepperMotor.__ser.write(('A'+str(float(accel))).encode())
            StepperMotor.__ser.reset_input_buffer()
            res = True
        except:
            res = False
        return res


    def getMotorSpeed(self):
        StepperMotor.__ser.write(('S\r').encode())
        self.d = StepperMotor.__ser.readline()
        self.p = self.d.find(('S').encode())
        self.q = self.d.find((' ').encode())
        self.d = self.d[self.p + 1 :]
        self.d = float(self.d) - 83.0
        return self.d

    def setMotorSpeed(self, speed):
        rclpy.logging._root_logger.debug('S'+str(int(speed)) + '\r')
        StepperMotor.__ser.reset_output_buffer()
        StepperMotor.__ser.write(('S'+str(int(speed))+ '\r').encode())
        StepperMotor.__ser.reset_input_buffer()

    #Input: take position/#steps values of motor 
    #Output: None
    #Description: organize pos values in G<x>A<y>B<z>C string format, send it to arduino,
    #arduino then decode pos values and control motor accordingly
    def setAbsolutePosition(self, position1, position2, position3):
        try:
            rclpy.logging._root_logger.debug('G'+(str(position1)+'A')+(str(position2)+'B')+(str(position3)+'C'))
            StepperMotor.__ser.reset_output_buffer()
            StepperMotor.__ser.write(('G'+(str(position1)+'A')+(str(position2)+'B')+(str(position3)+'C')).encode())
            StepperMotor.__ser.reset_input_buffer()
        except:
            pass
    
    def getMotorMicrostep(self):
        return MICROSTEP
    
    #Input: None 
    #Output: acknowledgement in boolean of sent msg
    #Description: send 'X' to arduino for calibration and set default value of motor i.e. speed and accel.
    def autoCalibrate(self):
        try:
            StepperMotor.__ser.write(('X\r').encode())
            res = True
        except:
            res = False
            pass
        return res

    #Input: None 
    #Output: acknowledgement in boolean of sent msg
    #Description: send 'X' to arduino for calibration and arduino set default value of motor i.e. speed and acceleration.
    def isAvailable(self):
        StepperMotor.__ser.reset_input_buffer()
        StepperMotor.__ser.write(('X').encode())
        time.sleep(1)
    
    def testTriangle(self,cnt):
        StepperMotor.__ser.reset_input_buffer()
        StepperMotor.__ser.write(('T'+(str(cnt))).encode())
        time.sleep(1)

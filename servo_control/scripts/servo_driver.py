#!/usr/bin/python3

import serial
import rclpy
import time
import six
from rclpy.impl.rcutils_logger import RcutilsLogger as log
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity

class ServoMotor:
    __port = ''
    __ser = serial.Serial(timeout=1, baudrate=9600)

    def __init__(self, serial_port_):
        ServoMotor.__port = serial_port_

    def openSerialPort(self):
        ServoMotor.__ser.port = ServoMotor.__port
        try:
            ServoMotor.__ser.open()
            rclpy.logging._root_logger.info('Opened the port'+ ServoMotor.__port)
        except:
            rclpy.logging._root_logger.fatal('Cannot open the port '+ ServoMotor.__port)
            rclpy.shutdown()

    def closeSerialPort(self):
        ServoMotor.__ser.close()

    def readMotorSpeed(self):
        ServoMotor.__ser.write(('S\r').encode())
        self.d = ServoMotor.__ser.readline()
        self.p = self.d.find(('S').encode())
        self.q = self.d.find((' ').encode())
        self.d = self.d[self.p + 1 : self.q]
        self.d = int(self.d)
        return self.d

    def writeMotorSpeed(self, speed):
        rclpy.logging._root_logger.debug('S'+str(int(speed)) + '\r')
        ServoMotor.__ser.reset_output_buffer()
        ServoMotor.__ser.write(('S'+str(int(speed))+ '\r').encode())
        ServoMotor.__ser.reset_input_buffer()

    def getMaxMotorSpeed(self):
        self.p = 0
        self.q = 0
        self.firstPass = True
        while self.p is -1 or self.q is -1 or self.firstPass:
            self.firstPass = False
            ServoMotor.__ser.reset_input_buffer()
            ServoMotor.__ser.write(('M\r').encode())
            try:
                self.d = ServoMotor.__ser.readline()
            except:
                pass
            self.p = self.d.find(('M').encode())
            self.q = self.d.find((' ').encode())
        self.d = self.d[self.p + 1 : self.q]
        self.d = int(self.d)
        ServoMotor.__ser.reset_input_buffer()
        return self.d

    def setMaxMotorSpeed(self, speed):
        try:
            ServoMotor.__ser.write(('M'+str(int(speed))).encode())
            res = True
        except:
            res = False
            pass
        return res

    def readDamping(self):
        try:
            ServoMotor.__ser.write('D\r')
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find(('D').encode())
            self.q = self.d.find((' ').encode())
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def writeDamping(self, value):
        try:
            ServoMotor.__ser.write(('D'+str(int(value))).encode())
            res = True
        except:
            res = False
            pass
        return res

    def loadFactorySettings(self):
        try:
            ServoMotor.__ser.write(('Y').encode())
            res = True
        except:
            res = False
            pass
        return res

    def getPositionEncoder(self):
        self.p = 0
        self.q = 0
        self.d = ''
        self.firstPass = True
        while self.p is -1 or self.q is -1 or self.firstPass:
            self.firstPass = False
            ServoMotor.__ser.reset_input_buffer()
            ServoMotor.__ser.write(('P\r').encode())
            try:
                self.d = ServoMotor.__ser.readline()
                self.p = self.d.find(('P').encode())
                self.q = self.d.find((' ').encode())
            except:
                return 0
        try:
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            if not self.d or isinstance(self.d, six.string_types):
                return 0
            return self.d
        except:
            return 0

    def setPositionEncoder(self, encoder):
        try:
            ServoMotor.__ser.write(('P'+str(encoder) + '\r').encode())
            res = True
        except:
            res = False
            pass
        return res

    def getAbsolutePostion(self):
        try:
            ServoMotor.__ser.write(('G\r').encode())
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find(('G').encode())
            self.q = self.d.find((' ').encode())
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setAbsolutePostion(self, position):
        try:
            ServoMotor.__ser.write(('G'+str(int(position)) + '\r').encode())
            res = True
        except:
            res = False
            pass
        return res

    def setRelativePostion(self, position):
        try:
            rclpy.logging._root_logger.debug('R'+str(int(position)))
            ServoMotor.__ser.reset_output_buffer()
            ServoMotor.__ser.write(('R'+str(int(position)) + '\r').encode())
            ServoMotor.__ser.reset_input_buffer()
        except:
            pass

    def getFeedbackGain(self):
        try:
            ServoMotor.__ser.write(('A\r').encode())
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find(('A').encode())
            self.q = self.d.find((' ').encode())
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setFeedbackGain(self, gain):
        try:
            ServoMotor.__ser.write(('A'+str(gain)).encode())
            res = True
        except:
            res = False
            pass
        return res

    def getProportionateGain(self):
        try:
            ServoMotor.__ser.write(('B\r').encode())
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find(('B').encode())
            self.q = self.d.find((' ').encode())
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setProportionateGain(self, gain):
        try:
            ServoMotor.__ser.write(('B'+str(gain)).encode())
            res = True
        except:
            res = False
            pass
        return res

    def getIntegralGain(self):
        try:
            ServoMotor.__ser.write(('C\r').encode())
            self.d = ServoMotor.__ser.readline()
            self.p = self.d.find(('C').encode())
            self.q = self.d.find((' ').encode())
            self.d = self.d[self.p + 1 : self.q]
            self.d = int(self.d)
            return self.d
        except:
            pass

    def setIntegralGain(self, gain):
        try:
            ServoMotor.__ser.write(('C'+str(gain)).encode())
            res = True
        except:
            res = False
            pass
        return res

    def autoCalibrate(self):
        try:
            ServoMotor.__ser.write(('X\r').encode())
            res = True
        except:
            res = False
            pass
        return res

    def isAvailable(self):
        ServoMotor.__ser.reset_input_buffer()
        ServoMotor.__ser.write(('P0\r').encode())
        time.sleep(1)

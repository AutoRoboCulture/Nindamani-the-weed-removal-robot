#!/usr/bin/python3

import queue
import threading
import time
import sys
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String, Int32
import cv2
import numpy as np
from perspectiveImage import perspective
from os import listdir
from xml.etree import ElementTree
from numpy import zeros
from numpy import asarray
from numpy import expand_dims
from mrcnn.config import Config
from mrcnn.model import MaskRCNN
from mrcnn.model import mold_image
from mrcnn.utils import Dataset
import skimage
import time
from matplotlib.patches import Rectangle
import pdb
from matplotlib import pyplot

#Input Output Files path
PRETRAINED_WEIGHT_PATH = 'src/rpicam_ai_interface/preTrained_weights/mask_rcnn_trained_weed_model.h5'
OUTPUT_IMAGE_PATH = '/src/rpicam_ai_interface/pointedWeedImg.jpg'
INPUT_TEST_IMAGE_PATH = '/src/rpicam_ai_interface/testImage.jpg'

class PredictionConfig(Config):
    # define the name of the configuration
        NAME = "weed_cfg"
        # number of classes (background + crop + weed)
        NUM_CLASSES = 1 + 2
        # simplify GPU config
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1


class Rpicam_ai_node:

    def __init__(self):
        
        #input: None
        #output: None
        #Description: Used for initializing all default values for hardware
        
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node('rpicam_ai_node', allow_undeclared_parameters=True)
        
        self.wb_mode = self.node.get_parameter_or(self.node.get_namespace() + 'wbmode', 1)
        self.exposure_compensation = self.node.get_parameter_or(self.node.get_namespace() + 'exposurecompensation', 0)
        self.rotateflip = self.node.get_parameter_or(self.node.get_namespace() + 'camera_rotate_flip', 0)       #flip_method
        self.image_width = self.node.get_parameter_or(self.node.get_namespace() + 'camera_image_width', 720)
        self.image_height = self.node.get_parameter_or(self.node.get_namespace() + 'camera_image_height', 720)
        self.saturation = self.node.get_parameter_or(self.node.get_namespace() + 'saturation', 1)
        
        #Model Input and Output Node Names
        self.input_names = ['input_image']
        self.output_names = ['rpn_bbox/concat']

        self.initialize_publisher()
        
        self.initialize_model()
        
        #Initiallizing the camera
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(self.wb_mode,self.saturation,self.image_width, self.image_height, 720, 720, 21, self.rotateflip), cv2.CAP_GSTREAMER)
        
        rclpy.spin(self.node)
     
    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self.gstreamer_pipeline(self.wb_mode,self.saturation,self.image_width, self.image_height, 720, 720, 21, self.rotateflip), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread = threading.Thread(target=self.take_pictures)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            self.thread.join()


    def destroy_node(self):
        super().destroy_node()

    def initialize_publisher(self):
        #Publish the object location to "objectcoordinates" topic
        self.publisher = self.node.create_publisher(Float32MultiArray, 'objectcoordinates',10)
        
        #Giving command for taking the pic or not
        self.sub = self.node.create_subscription(String,'takePicAgain', self.pictureCallback,10)
        self.manualSub = self.node.create_subscription(String,'manualWeedLoc', self.manualWeedLocation,10)

    def pictureCallback(self,msg):
        #input: Take data published by 'takePicAgain' topic
        #output: None
        #Description: Used for controlling when to take pic from camera
        
        self.inp = msg.data
        self.initialize_capture_queue()
    
    #Nvidia Specific function opening up the camera
    def gstreamer_pipeline(self,wb_mode,saturation,capture_width, capture_height, display_width, display_height, framerate, flip_method) :   
       return ('nvarguscamerasrc wbmode=(int)%d, saturation=(float)%f ! ' 
                'video/x-raw(memory:NVMM), '
                'width=(int)%d, height=(int)%d, '
                'format=(string)NV12, framerate=(fraction)%d/1 ! '
                'nvvidconv flip-method=%d ! '
                'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
                'videoconvert ! '
                'video/x-raw, format=(string)BGR ! appsink'  % (wb_mode,saturation,capture_width,capture_height,framerate,flip_method,display_width,display_height))

    def initialize_model(self):
        # create config
        self.cfg = PredictionConfig()
        # define the model
        self.model = MaskRCNN(mode='inference', model_dir='./', config=self.cfg)
        # load model weights
        model_path = PRETRAINED_WEIGHT_PATH
        self.model.load_weights(model_path, by_name=True)
        self.node.get_logger().info('MODEL IS READY NOW')

        
    def initialize_capture_queue(self):
         
        #Condition for taking the pic again
        if self.inp=='y':

            #Starting the Camera
            self.take_pictures()
        else:
            #Stopping the Camera
            self.stop()
    
    def weedLocation(self,image):
        #input: Take a input image provided by takePictures() function
        #output: Returns the number of weeds and its location
        #Description: Used to convert the detected pixel location of weeds into real distance by some measurement factor
        
        HEIGHT  =  -500.0   #Physically need to decide the ground height

        # convert image into one sample
        sample = expand_dims(image, 0)
        #cv2.imwrite("sample1.jpg",image)
        weedLoc = []
        weedCount = 0
        xFactor =  0.4688 #At HEIGHT -490 mm(from above), width(x-axis) 240mm(actual width that camera can capture) , 512x512 pixels (image size) --> 240/512 
        yFactor =  0.4688 #length(y-axis) 240mm --> 240/512
        # make prediction

        yhat = self.model.detect(sample, verbose=0)[0]
        
        boxCount = 0
        for classId in yhat['class_ids']:
            if (classId == 2): #classId 2 belongs to 'weed', classId 1 belongs to 'crop'
                box = yhat['rois'][boxCount]
                # get coordinates
                y1, x1, y2, x2 = box
        
                # calculate width and height of the box
                width, height = x2 - x1, y2 - y1
        
                # create the shape
                rect = Rectangle((x1, y1), width, height, fill=False, color='red')
                center = [(x1+(width/2)),(y1+(height/2))]
        
                #Shifted origin
                #To co-align camera and delta-Robot center, camera origin(256,256) shifted to delta robot origin(256,259):(x,y)  
                #with condition camera is fixed 
                cX1 = 260 - center[1] #259 value of y coordinate
                cY1 = -(center[0] - 257) #256 value of x coordinate
                cordis = tuple([(cX1*xFactor), (cY1*yFactor), HEIGHT])
                weedLoc.append(cordis)
                cv2.circle(image, (int(center[0]), int(center[1])), 3, (0, 255, 255), -1)
                cv2.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 2)
            boxCount+=1
        
        weedCount = len(weedLoc)
                
        if weedCount != 0:  
            #Output image storage path
            flnm = OUTPUT_IMAGE_PATH 
            cv2.imwrite(flnm, image)
            weedLoc = np.reshape(weedLoc, (1, weedCount*3))
        else:
            data = tuple([0.0,0.0,-280.0])
            weedLoc.append(data)
            weedLoc = np.reshape(weedLoc, (1, 3))
    
        return weedCount,weedLoc

    def manualWeedLocation(self,msg):
        #input: Take a input weed data location in form of G100A-20B-430C->(x,y,z) provided by 'manualWeedLoc' topic
        #output: None
        #Description: Used for testing purpose by providing location, the delta arm goes to that location or not
        
        cordData = msg.data
        cordA = cordData.find("A")
        cordB = cordData.find("B")
        position1 = float(cordData[1:cordA])
        position2 = float(cordData[cordA+1:cordB])
        position3 = float(cordData[cordB+1:-1])
        
        weedLoc = []
        weedCount = 1
        if weedCount != 0:
            data = tuple([position1,position2,position3])
            weedLoc.append(data)
            weedLoc = np.reshape(weedLoc, (1, weedCount*3))
        else:
            data = tuple([0.0,0.0,-280.0])
            weedLoc.append(data)
            weedLoc = np.reshape(weedLoc, (1, 3))
   
        objCnt = weedCount;
        objCord = weedLoc;
        #Making up the object location data in format for publishing
        if objCnt !=0:
            objCord = np.reshape(objCord,(objCnt*3,))  #reshaping in the form of single column
            objCord = np.ndarray.tolist(objCord)
        else:
            objCord = np.reshape(objCord,(3,))  #reshaping in the form of single column
            objCord = np.ndarray.tolist(objCord)
                
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        msg.layout.dim[0].label = "row"
        msg.layout.dim[1].label = "col"
        msg.layout.dim[0].size = objCnt
        msg.layout.dim[1].size = 1
        msg.layout.dim[0].stride = 3
        msg.layout.dim[1].stride = 1
        msg.layout.data_offset = 0        
        msg.data = objCord
        self.publisher.publish(msg)


    def take_pictures_test(self):
        #input: None
        #output: None
        #Description: Used for validating AI is working with camera or not
        
        #Getting the bird eye view of the image 
        #persImg = perspective(cv_image)
        persImg = skimage.io.imread(INPUT_TEST_IMAGE_PATH)
                 
        objCnt,objCord  = self.weedLocation(persImg)
        #Making up the object location data in format for publishing

        if objCnt !=0:
                objCord = np.reshape(objCord,(objCnt*3,))  #reshaping in the form of single column
                objCord = np.ndarray.tolist(objCord)
        else:
                objCord = np.reshape(objCord,(3,))  #reshaping in the form of single column
                objCord = np.ndarray.tolist(objCord)
       
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        msg.layout.dim[0].label = "row"
        msg.layout.dim[1].label = "col"
        msg.layout.dim[0].size = objCnt
        msg.layout.dim[1].size = 1
        msg.layout.dim[0].stride = 3
        msg.layout.dim[1].stride = 1
        msg.layout.data_offset = 0        
        msg.data = objCord
        self.publisher.publish(msg)

    def take_pictures(self):
        #input: None
        #output: None
        #Description: Used for capturing a single frame from camera
        if self.cap.isOpened():
            cnt =0 

            #For removing the shadow pic, taking pic after some time
            while True:
                ret_val, cv_image = self.cap.read()
                time.sleep(1)
                if cnt == 1:
                    break
                cnt=cnt+1

            if ret_val:

                #Getting the bird eye view of the image 
                persImg = perspective(cv_image)
                objCnt,objCord  = self.weedLocation(persImg)
                #Predicting the object location
                #Making up the object location data in format for publishing
                if objCnt !=0:
                    objCord = np.reshape(objCord,(objCnt*3,))  #reshaping in the form of single column
                    objCord = np.ndarray.tolist(objCord)
                else:
                    objCord = np.reshape(objCord,(3,))  #reshaping in the form of single column
                    objCord = np.ndarray.tolist(objCord)
                
                msg = Float32MultiArray()
                msg.layout.dim.append(MultiArrayDimension())
                msg.layout.dim.append(MultiArrayDimension())

                msg.layout.dim[0].label = "row"
                msg.layout.dim[1].label = "col"
                msg.layout.dim[0].size = objCnt
                msg.layout.dim[1].size = 1
                msg.layout.dim[0].stride = 3
                msg.layout.dim[1].stride = 1
                msg.layout.data_offset = 0        
                msg.data = objCord
                if objCnt !=0: 
                    self.publisher.publish(msg)
            else:

                self.node.get_logger().info('Not able to read Image!!!!!!')

        else:
            print ('Unable to open camera')
    

def main(args=None):
    camNode = Rpicam_ai_node()

if __name__ == '__main__':
    main()

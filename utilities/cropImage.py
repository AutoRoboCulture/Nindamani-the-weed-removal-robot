import numpy as np
import cv2
import matplotlib.pyplot as plt
import pdb
import glob
import os
from PIL import Image

#Crop Images 

#Input and Output Path for Images

def cropFunc(crop_height_top, crop_width):
    #Count Number of Images     
    input_path = '<input image path>/*.jpg'
    
    #```Temp Output Path```#
    output_path = "<output image save path>"

    name = input_path.replace('*.jpg','')
    path, dirs, files = next(os.walk(name))
    file_count = len(files)

    cnt = 0
    jpg_name = []
    output_path = output_path+'%s'
    
    crp_h = (100 - crop_height_top)/100
    crp_w = crop_width/100
    for img_jpg in glob.glob(input_path):       #give directory path of origianl images dataset
            read = cv2.imread(img_jpg)
            img_name = img_jpg.replace(name,'')
            filename = output_path%img_name        #output images directory path
            h,w,c = read.shape
            h1 = int(h - (crp_h*h)) #% height, from bottom side
            w2 = int(w - (crp_w*w)) # leave right )side
            w1 = int(crp_w*w) #leave left side
            crop_img = read[h1:h,w1:w2]
            cv2.imwrite(filename,crop_img)
            cnt+=1
            print ('Cropped: ',cnt,'Out of: ',file_count)
    print (cnt,' Images Cropped\n')

#Function Calling
#(height,width) values in percentage, 
#Note: example. (0,30) means crop 30% from right side and 30% from left side
cropFunc(0,30)  

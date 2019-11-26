from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing.image import load_img, img_to_array
from matplotlib import pyplot as plt
import glob
import os
import numpy as np
import pdb

def imageAugment():
   #pdb.set_trace()
   lenAug = 2
   #***************************Image Dir Path**************************************#
   groundRidgePath      ="<input_to_path_image_folder>/*.jpg"
   saveGrndRidgeDir     ="<output_folder_path>/"
   
   #***************************Loading Image**************************************#
   groundRidgeImg    = glob.glob(groundRidgePath)
   
   groundRidgeImg.sort()
   
   #***************************Generating Train groundRidgeImage Augmentation***********#
   if bool(groundRidgeImg):
      
 
      groundRidgeDatagen     = ImageDataGenerator(horizontal_flip=True, channel_shift_range=50.0)
      
      cnt = 1
      path, dirs, files = next(os.walk(groundRidgePath.split("*")[0]))
      file_count = len(files)
      if file_count%2!=0:
          while((file_count%2)!=0):
              file_count+=1
      imgList = []
      for img in groundRidgeImg:
          mat = load_img(img,target_size=(1024,1024))
          mat = img_to_array(mat)
          imgList.append(mat)
      imgList = np.array(imgList)
      
      #Augmented images = original images * batch_size  //(original images should be in even number)
      iterator = groundRidgeDatagen.flow(imgList, 
                              y=None, 
                              batch_size=2,  
                              shuffle=True, 
                              sample_weight=None, 
                              seed=88,
                              save_to_dir=saveGrndRidgeDir, 
                              save_prefix='', 
                              save_format='jpg', 
                              subset=None)
      
      for e in range(0,file_count): # in case the generator is null
           iterator.next()
           print ('Processed groundRidgeAugmented Images: ',cnt,'Out of: ',file_count)
           cnt+=1

   #**************************************END***************************************#


imageAugment()

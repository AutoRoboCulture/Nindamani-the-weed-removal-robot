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
   groundRidgePath      ="../dataset/fresh/*.jpg"
   #groundNonRidgePath   ="../dataset/newComer/groundTruth/nonRidge/*.png"
   #trainRidgePath       ="../dataset/newComer/ridge/*.jpg"
   #trainNonRidgePath    ="../dataset/newComer/nonRidge/*.jpg"
   
   saveGrndRidgeDir     ="../dataset/augmented/"
   #saveRidgeDir         ="../dataset/newComer/dataAugment/x/ridge/"
   #saveGrndNonRidgeDir  ="../dataset/newComer/dataAugment/y/nonRidge/"
   #saveNonRidgeDir      ="../dataset/newComer/dataAugment/x/nonRidge/"
   
   #***************************Loading Image**************************************#
   groundRidgeImg    = glob.glob(groundRidgePath)
   #trainRidgeImg     = glob.glob(trainRidgePath)
   
   #groundNonRidgeImg = glob.glob(groundNonRidgePath)
   #trainNonRidgeImg  = glob.glob(trainNonRidgePath)

   groundRidgeImg.sort()
   #trainRidgeImg.sort()
   #groundNonRidgeImg.sort()
   #trainNonRidgeImg.sort()

   #***************************Generating Train groundRidgeImage Augmentation***********#
   if bool(groundRidgeImg):
      
 
      #groundRidgeDatagen    = ImageDataGenerator(zoom_range=[0.8,1],horizontal_flip=True,brightness_range=(0.8,1.0))
      #groundRidgeDatagen     = ImageDataGenerator(zoom_range=[0.5,1],horizontal_flip=True,vertical_flip = True, brightness_range=[0.1,1.0],rotation_range=360)
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

   #***************************Generating TrainRidgeImage Augmentation*******************#
   '''
      cnt = 1
      path, dirs, files = next(os.walk(trainRidgePath.split("*")[0]))
      file_count = len(files)
      if file_count%4!=0:
          while((file_count%4)!=0):
              file_count+=1
      
      imgList = []
      for img in trainRidgeImg:
          mat = load_img(img,target_size=(224,224))
          mat = img_to_array(mat)
          imgList.append(mat)
      imgList = np.array(imgList)
      iterator = trainRidgeDatagen.flow(imgList, 
                             y=None, 
                             batch_size=4, 
                             shuffle=True, 
                             sample_weight=None, 
                             seed=88, 
                             save_to_dir=saveRidgeDir, 
                             save_prefix='', 
                             save_format='jpg', 
                             subset=None)
      for e in range(0,file_count): # in case the generator is null
           iterator.next()
           print ('Processed trainRidgeAugmented Images: ',cnt,'Out of: ',file_count)
           cnt+=1

   else:
      print("No Ridge Image found in groundTruth & ridge folder")
   
   #***************************Generating Train groundNonRidgeImage Augmentation********#
   if bool(groundNonRidgeImg) and bool(trainNonRidgeImg):
      groundNonRidgeDatagen = ImageDataGenerator(zoom_range=[0.8,1],horizontal_flip=True,brightness_range=(0.8,1.0))
      trainNonRidgeDatagen  = ImageDataGenerator(zoom_range=[0.8,1],horizontal_flip=True,brightness_range=(0.8,1.0))
   
      cnt = 1
      path, dirs, files = next(os.walk(groundNonRidgePath.split("*")[0]))
      file_count = len(files)
      if file_count%4!=0:
          while((file_count%4)!=0):
              file_count+=1
      
      imgList = []
      for img in groundNonRidgeImg:
          mat = load_img(img,target_size=(224,224),color_mode='grayscale')
          mat = img_to_array(mat)
          imgList.append(mat)
      imgList = np.array(imgList)
      
      iterator = groundNonRidgeDatagen.flow(imgList, 
                                 y=None, 
                                 batch_size=4, 
                                 shuffle=True, 
                                 sample_weight=None, 
                                 seed=88, 
                                 save_to_dir=saveGrndNonRidgeDir, 
                                 save_prefix='', 
                                 save_format='png', 
                                 subset=None)
      
      for e in range(0,file_count): # in case the generator is null
           iterator.next()
           print ('Processed groundNonRidgeAugmented Images: ',cnt,'Out of: ',file_count)
           cnt+=1
   
   #***************************Generating Train NonRidgeImage Augmentation********#
      cnt = 1
      path, dirs, files = next(os.walk(trainNonRidgePath.split("*")[0]))
      file_count = len(files)
      if file_count%4!=0:
          while((file_count%4)!=0):
              file_count+=1
      imgList = []
      
      for img in trainNonRidgeImg:
          mat = load_img(img,target_size=(224,224))
          mat = img_to_array(mat)
          imgList.append(mat)
      imgList = np.array(imgList)
   
      iterator = trainNonRidgeDatagen.flow(imgList, 
                                y=None, 
                                batch_size=4, 
                                shuffle=True, 
                                sample_weight=None, 
                                seed=88, 
                                save_to_dir=saveNonRidgeDir, 
                                save_prefix='', 
                                save_format='jpg', 
                                subset=None)
      for e in range(0,file_count): # in case the generator is null
           iterator.next()
           print ('Processed trainNonRidgeAugmented Images: ',cnt,'Out of: ',file_count)
           cnt+=1
   else:
      print("No nonRidge Image found in groundTruth & ridge folder")
   
   print("**************************DONE***************************************")
   return
   '''
#**************************************END***************************************#

imageAugment()

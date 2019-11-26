import glob
import pdb
import os
import re

#pdb.set_trace()

#List of functions:
#1) renameImg()
#2) renameDAImg()
#3) renameFresh()

#<------------------Use: For renaming image and rewrite in same folder by deleting original name image------------>#
def renameImg():
        
    #Input path
    ridgeInput_path = '../dataset/newComer/ridge/*.jpg'
    nonRidgeInput_path = '../dataset/newComer/nonRidge/*.jpg'
    lineDrawnInput_path = '../dataset/newComer/lineDrawn/*.png'

    #Save Location
    ridgeOutput_path = '../dataset/newComer/ridge/'
    nonRidgeOutput_path = '../dataset/newComer/nonRidge/'
    lineDrawnOutput_path = '../dataset/newComer/lineDrawn/'
    
    #TrainX and Y path for getting last saved number
    ridgeTrainXY_path = '../dataset/trainX/ridge/'     #To get last image image_#number in Dataset
    pathtR, dirstR, filestR = next(os.walk(ridgeTrainXY_path))
    file_counttR = len(filestR)
    nonRidgeTrainXY_path = '../dataset/trainX/nonRidge/'     #To get last image image_#number in Dataset
    pathtNR, dirstNR, filestNR = next(os.walk(nonRidgeTrainXY_path))
    file_counttNR = len(filestNR)
    
    
    #Get file count in folder
    nameR = ridgeInput_path.replace('*.jpg','')
    pathR, dirsR, filesR = next(os.walk(nameR))
    file_countR = len(filesR)
    
    nameNR = nonRidgeInput_path.replace('*.jpg','')
    pathNR, dirsNR, filesNR = next(os.walk(nameNR))
    file_countNR = len(filesNR)
    
    nameLD = lineDrawnInput_path.replace('*.png','')
    pathLD, dirsLD, filesLD = next(os.walk(nameLD))
    file_countLD = len(filesLD)
   #````````````````````````````````Natural Sort Function`````````````````````````````#
    def natural_sort(l): 
        convert = lambda text: int(text) if text.isdigit() else text.lower() 
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
        return sorted(l, key = alphanum_key)
    
    #```````````````````````````````Name Change Function````````````````````````````````#
    def nC(input_path, output_path, file_cnt, file_name):
        
        img_num = file_cnt
        #img_num = 0

        #File Count
        imgExt = file_name.split(".")[1]
        name = input_path.replace('*.'+imgExt,'')
        path, dirs, files = next(os.walk(name))
        file_count = len(files)
        
        cnt = 0
        del_img = []
        output_path = output_path+file_name     #Output Image Filename
        
        sort_temp = []
        for img_jpg1 in glob.glob(input_path):
            sort_temp.append(img_jpg1)
        sort = natural_sort(sort_temp)

        for img_jpg in sort:

            del_img.append(img_jpg)     #Keeping orginal image name for deletion at the end
            #read = cv2.imread(img_jpg)       #read image
            filename = output_path%int(img_num)        #Output Image filename
            os.rename(img_jpg, filename)
            #cv2.imwrite(filename,read)       #save image file
            cnt+=1
            img_num+=1
            print ('Renamed: ',cnt,'Out of: ',file_count)
        
        #Delete original images
        #for dI in del_img:
        #    os.remove(dI)
    
    #`````````````````````````````Function Calling```````````````````````````````````#

    if file_countR:
        file_name = 'ridge%05d.jpg'
        print('Ridges ---->')
        nC(ridgeInput_path, ridgeOutput_path, file_counttR, file_name)
    else: 
        print('Ridges ---->')
        print (nameR, 'folder is Empty!!\n')
   

    if file_countNR:
        file_name = 'nonRidge%05d.jpg'
        print ('NonRidges ---->')
        nC(nonRidgeInput_path, nonRidgeOutput_path, file_counttNR, file_name)
    else: 
        print ('\nNonRidges ---->')
        print (nameNR, 'folder is Empty!!\n')
   
    if file_countLD:
        file_name = 'ridge%05d.png'
        print ('LineDrawn_Ridges ---->')
        nC(lineDrawnInput_path, lineDrawnOutput_path, file_counttR, file_name)
    else: 
        print ('\nLineDrawn_NonRidges ---->')
        print (nameLD, 'folder is Empty!!\n')
    
    print('<---Done--->')
    return
#################################..........END OF def renameImg...........##################################


#<---------------------------------Use: For renaming dataAugment Image-------------------------->#
def renameDAImg():
    #pdb.set_trace()
    
    #``````````````````````````````````Input path```````````````````````````````````````#

    ifileXR = '../dataset/newComer/dataAugment/x/ridge/*.jpg'
    ifileXNR = '../dataset/newComer/dataAugment/x/nonRidge/*.jpg'
    ifileYR = '../dataset/newComer/dataAugment/y/ridge/*.png'
    ifileYNR = '../dataset/newComer/dataAugment/y/nonRidge/*.png'

    #`````````````````````````````````Save Location```````````````````````````````````````#

    ofileXR = '../dataset/newComer/dataAugment/x/ridge/'
    ofileXNR = '../dataset/newComer/dataAugment/x/nonRidge/'
    ofileYR = '../dataset/newComer/dataAugment/y/ridge/'
    ofileYNR = '../dataset/newComer/dataAugment/y/nonRidge/'
    
    #```````````````````TrainX and Y path for getting last saved number````````````````````#

    ridgeTrainXY_path = '../dataset/trainX/ridge/'     #To get last image image_#number in Dataset
    pathtR, dirstR, filestR = next(os.walk(ridgeTrainXY_path))
    file_counttR = len(filestR)
    nonRidgeTrainXY_path = '../dataset/trainX/nonRidge/'     #To get last image image_#number in Dataset
    pathtNR, dirstNR, filestNR = next(os.walk(nonRidgeTrainXY_path))
    file_counttNR = len(filestNR)
    
    
    #`````````````````````````Get file count in folder``````````````````````````````````````#

    nameXR = ifileXR.replace('*.jpg','')
    pathXR, dirsXR, filesXR = next(os.walk(nameXR))
    file_countXR = len(filesXR)
    
    nameXNR = ifileXNR.replace('*.jpg','')
    pathXNR, dirsXNR, filesXNR = next(os.walk(nameXNR))
    file_countXNR = len(filesXNR)
    
    nameYR = ifileYR.replace('*.png','')
    pathYR, dirsYR, filesYR = next(os.walk(nameYR))
    file_countYR = len(filesYR)
    
    nameYNR = ifileYNR.replace('*.png','')
    pathYNR, dirsYNR, filesYNR = next(os.walk(nameYNR))
    file_countYNR = len(filesYNR)
    

    #``````````````````````````````Natural Sort Function````````````````````````````````````````#
    def natural_sort(l): 
        convert = lambda text: int(text) if text.isdigit() else text.lower() 
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
        return sorted(l, key = alphanum_key)
    
    #``````````````````````````````Name Change Function````````````````````````````````````````````#
    def nC(input_path, output_path, file_cnt, file_name):
        
        img_num = file_cnt

        #File Count
        imgExt = file_name.split(".")[1]
        name = input_path.replace('*.'+imgExt,'') 
        path, dirs, files = next(os.walk(name))
        file_count = len(files)
        
        cnt = 0
        del_img = []
        output_path = output_path+file_name     #Output Image Filename
        
        sort_temp = []
        for img_jpg1 in glob.glob(input_path):
            sort_temp.append(img_jpg1)
        sort = natural_sort(sort_temp)

        for img_jpg in sort:

            del_img.append(img_jpg)     #Keeping orginal image name for deletion at the end
            #read = cv2.imread(img_jpg)       #read image
            filename = output_path%int(img_num)        #Output Image filename
            os.rename(img_jpg, filename)
            #cv2.imwrite(filename,read)       #save image file
            cnt+=1
            img_num+=1
            print ('Renamed: ',cnt,'Out of: ',file_count)
        
        #Delete original images
        #for dI in del_img:
        #    os.remove(dI)

    #```````````````````````````````Function Calling``````````````````````````````#

    if file_countXR:
        file_name = 'ridge%05d.jpg'
        print('DataAug_X_Ridges ---->')
        nC(ifileXR, ofileXR, file_counttR, file_name)
    else: 
        print('DataAug_X_Ridges ---->')
        print (nameXR, 'folder is Empty!!\n')
   

    if file_countXNR:
        file_name = 'nonRidge%05d.jpg'
        print ('DataAug_X_NonRidges ---->')
        nC(ifileXNR, ofileXNR, file_counttNR, file_name)
    else: 
        print ('\nDataAug_X_NonRidges ---->')
        print (nameXNR, 'folder is Empty!!\n')
    
    #____________________________________________________________#
    
    if file_countYR:
        file_name = 'ridge%05d.png'
        print('DataAug_Y_Ridges ---->')
        nC(ifileYR, ofileYR, file_counttR, file_name)
    else: 
        print('DataAug_Y_Ridges ---->')
        print (nameYR, 'folder is Empty!!\n')
   

    if file_countYNR:
        file_name = 'nonRidge%05d.png'
        print ('DataAug_Y_NonRidges ---->')
        nC(ifileYNR, ofileYNR, file_counttNR, file_name)
    else: 
        print ('\nDataAug_Y_NonRidges ---->')
        print (nameYNR, 'folder is Empty!!\n')
    

    print('<---Done--->')
    return
#############################............End of def renameDAImg...........#################################################


#<----------------------------------Use: For temporary renaming any fresh input image from outside--------------------->#
def renameFresh(imgExt):
        
    #``````````````````````````````````````Input path``````````````````````````````````````#
    #ridgeInput_path = '../dataset/fresh/*.'+imgExt
    #ridgeInput_path = '../dataset/testDataset/*.'+imgExt
    #ridgeInput_path = '../dataset/augmented/*.'+imgExt
    #nonRidgeInput_path = '../dataset/boundingBox/*.'+imgExt
    ridgeInput_path = '/home/kevin/Desktop/outputImg/*.'+imgExt

    #``````````````````````````````````````Save Location``````````````````````````````````````#
    #ridgeOutput_path = '../dataset/renamedWeed/'
    #ridgeOutput_path = '../dataset/testDataset/renamed/'
    #ridgeOutput_path = '../dataset/renamedAugmented/'
    #nonRidgeOutput_path = '../dataset/newComer/nonRidge/'
    ridgeOutput_path = '/home/kevin/Desktop/outputImg/renamed/'
    
    #`````````````````````````TrainX and Y path for getting last saved number```````````````````#
    
    #ridgeTrainXY_path = '../dataset/trainX/ridge/'     #To get last image image_#number in Dataset
    #pathtR, dirstR, filestR = next(os.walk(ridgeTrainXY_path))
    #file_counttR = len(filestR)
    #nonRidgeTrainXY_path = '../dataset/trainX/nonRidge/'     #To get last image image_#number in Dataset
    #pathtNR, dirstNR, filestNR = next(os.walk(nonRidgeTrainXY_path))
    #file_counttNR = len(filestNR)
    
    
    #```````````````````````````Get file count in folder```````````````````````````````````````#

    nameR = ridgeInput_path.replace('*.'+imgExt,'')
    pathR, dirsR, filesR = next(os.walk(nameR))
    file_countR = len(filesR)
    
    #nameNR = nonRidgeInput_path.replace('*.'+imgExt,'')
    #pathNR, dirsNR, filesNR = next(os.walk(nameNR))
    #file_countNR = len(filesNR)
    
    #``````````````````````````````````Natural Sort Function````````````````````````````````````````#
    def natural_sort(l): 
        convert = lambda text: int(text) if text.isdigit() else text.lower() 
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
        return sorted(l, key = alphanum_key)
    
    #``````````````````````````````````Name Change Function````````````````````````````````````````#
    def nC(input_path, output_path, file_cnt, file_name):
        
        img_num = 1     #file number initial

        #File Count
        name = input_path.replace('*.'+imgExt,'')

        path, dirs, files = next(os.walk(name))
        file_count = len(files)
        
        cnt = 0
        del_img = []
        output_path = output_path+file_name     #Output Image Filename
        
        sort_temp = []
        for img_jpg1 in glob.glob(input_path):
            sort_temp.append(img_jpg1)
        sort = natural_sort(sort_temp)

        for img_jpg in sort:

            #del_img.append(img_jpg)     #Keeping orginal image name for deletion at the end
            #read = cv2.imread(img_jpg)       #read image
            filename = output_path%int(img_num)        #Output Image filename
            os.rename(img_jpg, filename)
            #cv2.imwrite(filename,read)       #save image file
            cnt+=1
            img_num+=1
            print ('Renamed: ',cnt,'Out of: ',file_count)
        
        #Delete original images
        #for dI in del_img:
        #    os.remove(dI)
    #`````````````````````````````````````Function Calling`````````````````````````````````````#

    if file_countR:
        #file_name = 'renamed'+'%05d.'+imgExt
        file_name = 'predictedWeed'+'%05d.'+imgExt
        print('Ridges ---->')
        nC(ridgeInput_path, ridgeOutput_path, 999, file_name)
    else: 
        print('Ridges ---->')
        print (nameR, 'folder is Empty!!\n')
   
    '''
    if file_countNR:
        file_name = 'nr%05d.'+imgExt
        print ('NonRidges ---->')
        nC(nonRidgeInput_path, nonRidgeOutput_path, file_counttNR, file_name)
    else: 
        print ('\nNonRidges ---->')
        print (nameNR, 'folder is Empty!!\n')
    '''
    
    print('<---Done--->')
    return
########################################......End of def renameFresh............#######################################################



#Function Calling
#renameDAImg()
#renameFresh('jpg')
renameFresh('png')

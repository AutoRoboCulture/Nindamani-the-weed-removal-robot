import numpy as np
import cv2 as cv

#Getting input/raw image from rpicam_ai_interface.py 
def perspective(rawImg):

    #input image storage path
    input_img = "src/rpicam_ai_interface/inputImg.jpg"
    
    #Perspected image storage path
    output_img = "src/rpicam_ai_interface/perspectedImg.jpg" 
    
    cv.imwrite(input_img,rawImg)
    input_img = cv.imread(input_img)
    input_img = cv.resize(input_img,(512,512))
 
    #-----USER INPUT REQUIRED-----------#
    #Manually getting this coordinates (from mouse_perspective.py)
    #Delta robot working region 
    #Take note image center coordinate from manully clicked image
    cordi_x = [121,347,390,112]
    cordi_y = [84,48,455,501]
    src_cord = np.float32([[cordi_x[0]+2,cordi_y[0]],[cordi_x[1]-2,cordi_y[1]],[cordi_x[2]-0.2,cordi_y[2]],[cordi_x[3]+0.2,cordi_y[3]]])  
    
    #Define width and height of perspected image
    w = cordi_x[2] - cordi_x[3]
    h = cordi_y[2] - cordi_y[1]
    dst_cord = np.float32([[0,0],[w,0],[w,h],[0,h]])
    M = cv.getPerspectiveTransform(src_cord,dst_cord)
    img_dst = cv.warpPerspective(input_img,M,(w,h),flags=cv.INTER_LINEAR)
    
    img_dst = cv.resize(img_dst, (512,512))     #enter the resize image output dimension
    cv.imwrite(output_img,img_dst)
    
    return img_dst

#if __name__=='__main__':
#    checkImg = cv.imread('<enter your input image path>')
#    ck = cv.resize(checkImg,(512,512))
#    perspective(ck)

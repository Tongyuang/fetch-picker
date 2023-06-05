#!/usr/bin/env python3

from ultralytics import YOLO
from PIL import Image
import numpy as np 
from perception.srv import *
from utils import *
import cv2
import rospy

class Segmentator:
    def __init__(self, model_pth = '/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/YOLOv8/weights/bestv0602.pt'):
        try:
            self.model = YOLO(model_pth)
        except:
            print('cannot load model at {}'.format(model_pth))
    
    def predict(self,image):
        """image should be transfered into 640x640x1

        Args:
            image: input image
        """
        # bbox_list,mask_list,process_time = detectObject(self.model,image)
        return detectObject(self.model,image)
    
    def drawSegmentResult(self,image,bbox_list,mask_list,process_time):
        if bbox_list and mask_list and process_time:
            for bounding_box in bbox_list:
                drawbbox(image,bounding_box)
            for mask in mask_list:
                drawmask(image,mask)

def camera_image_client():
    print('waiting for service...')
    rospy.wait_for_service('get_image')
    try:
        
        serv = rospy.ServiceProxy('get_image',GetImage)
        image = serv()  
        return image
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None   

if __name__ == "__main__":
    segmentator = Segmentator()
    ret = camera_image_client()
    if ret:
        image = ret.image
        image_np = np.frombuffer(image.data, np.uint8)
        image_np = np.reshape(image_np, (image.height, image.width, -1))
        # reshape into (1,3,w,h)
        #image_np = np.resize(image_np,(1,3,image.height, image.width))
        #print(image_np.shape)
        bbox_list,mask_list,process_time = segmentator.predict([image_np])
        print(bbox_list,mask_list,process_time)
    # test_img = cv2.imread('/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toolwall.png')
    # bbox_list,mask_list,process_time = segmentator.predict(test_img)
    # segmentator.drawSegmentResult(test_img,bbox_list,mask_list,process_time)
    # print(mask_list[0])
    #cv2.imwrite('/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toolwall_pred.png',test_img)
    # print(segmentator.model.predict('/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toolwall.png'))
    
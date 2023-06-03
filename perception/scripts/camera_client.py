#!/usr/bin/env python3

import rospy
import cv2
from perception.srv import *
import numpy as np

def show_image():
    ret = camera_image_client()
    if ret:
        image = ret.image
        image_np = np.frombuffer(image.data, np.uint8)
        image_np = np.reshape(image_np, (image.height, image.width, -1))
        # cut the image
        new_image_np = np.zeros((image.height,image.height,3))
        
        print('image_shape:{}'.format(image_np.shape))
        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        cv2.imshow('test',image_np)
        
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

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
    show_image()
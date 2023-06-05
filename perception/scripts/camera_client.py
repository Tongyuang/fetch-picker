#!/usr/bin/env python3

import rospy
import cv2
from perception.srv import *
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import time
from perception.utils import * 

def image_demo():
    ret = camera_image_client()
    if ret:
        image = ret.image
        image_np = np.frombuffer(image.data, np.uint8)
        image_np = np.reshape(image_np, (image.height, image.width, -1))
        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        
        cv2.imwrite('/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toowall.png',image_np)
    
def pointcloud_demo():
    ret = camera_cloud_client()
    if ret:
        print('ret received.')
        cloud = ret.pointcloud2
        points = pc2.read_points(cloud,field_names=("x","y","z"),skip_nans = False)
        points = list(points)
        print(len(points),points[0])
        savebag(cloud,save_dir='/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toolwall.bag')
    


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
    
def camera_cloud_client():
    print('waiting for service...')
    rospy.wait_for_service('get_image')
    print('server setup!')
    try:
        serv = rospy.ServiceProxy('get_cloud',GetCloud) 
        return serv()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None   

if __name__ == "__main__":
    #pointcloud_demo()
    cloud = readbag(save_dir='/usr/yuangtong/study/UWGIX/courses/TECHIN517A/perception/test/test_toolwall.bag')
    points = pc2.read_points(cloud,field_names=("x","y","z"),skip_nans = False)
    points = list(points)
    print(len(points))
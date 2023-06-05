'''
 # @ Author: Yuang Tong
 # @ Create Time: 2023-06-02 15:47:11
 # @ Modified by: Yuang Tong
 # @ Modified time: 2023-06-03 16:35:55
 # @ Description:
 '''
#! /usr/bin/env python3

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from perception.srv import (GetCloud, GetCloudResponse, GetImage,
                            GetImageResponse)
from sensor_msgs.msg import Image, PointCloud2

HEADCAMERA_TOPIC_RAWRGB = "/head_camera/rgb/image_raw"
POINTCLOUD_TOPIC = "/head_camera/depth_registered/points"

class CameraServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(HEADCAMERA_TOPIC_RAWRGB, Image, self.callback)
        self.pointCloudsub = rospy.Subscriber(POINTCLOUD_TOPIC, PointCloud2, self.pointCloudCallback)
        self.last_image,self.last_pointcloud = None,None
        self.image_service = rospy.Service('get_image',GetImage,self.image_server_callback)
        self.cloud_service = rospy.Service('get_cloud',GetCloud,self.cloud_server_callback)
    def pointCloudCallback(self,msg):
        points = pc2.read_points(msg,field_names=("x","y","z"),skip_nans = False)
        self.last_pointcloud = list(points)
        
    def callback(self,msg):
        self.last_image = msg
    
    def image_server_callback(self,request):
        return GetImageResponse(self.last_image)
    
    def cloud_server_callback(self,request):
        return GetCloudResponse(self.last_pointcloud)
    


def main():
    image_service = CameraServer()
    rospy.init_node('camera_server', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
        

        
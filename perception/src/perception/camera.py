#! /usr/bin/env python3

import rospy
import tf.transformations as tft
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from perception.srv import GetImage,GetImageResponse

HEADCAMERA_TOPIC_RAWRGB = "/head_camera/rgb/image_raw"

class CameraServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(HEADCAMERA_TOPIC_RAWRGB, Image, self.callback)
        self.last_image = None
        self.image_service = rospy.Service('get_image',GetImage,self.server_callback)
    def callback(self,msg):

        
        self.last_image = msg
    
    def server_callback(self,request):
        return GetImageResponse(self.last_image)


def main():
    image_service = CameraServer()
    rospy.init_node('camera_server', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
        

        
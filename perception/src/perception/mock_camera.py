#!/usr/bin/env python
'''
 # @ Author: Yuang Tong
 # @ Create Time: 2023-05-15 14:31:35
 # @ Modified by: Yuang Tong
 # @ Modified time: 2023-05-15 14:32:30
 # @ Description:
 '''
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
import os

class MockCamera():
    def __init__(self):
        pass
    
    def read_cloud(self,path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.
    
        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """ 
        
        if os.path.exists(path):
            default_topic = "head_camera/depth_registered/points"
            msg_val = PointCloud2()
            with rosbag.Bag(path,'r') as bag:
                for (topic,msg,time) in bag.read_messages(topics=[default_topic]):
                    msg_val = msg
            return msg_val
        return None

if __name__ == "__main__":
    camera = MockCamera()
    msg = camera.read_cloud('../../bagfiles/shelf.bag')
    print(msg.header)
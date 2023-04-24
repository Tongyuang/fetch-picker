#!/usr/bin/env python3

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA



class RvizMarker(object):
    """Add a marker display to Rviz
    """
    def __init__(self):
        # marker
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker,queue_size=5)
        # wait for 0.5 secs
        rospy.sleep(0.5)
        
        rospy.loginfo("marker publisher started.") 

    def show_text_in_rviz(self, text):
        MarkerPosition = Point(1,1,1.45)
        MarkerQuaternion = Quaternion(0,0,0,1)
        MarkerScale = Vector3(0.5,0.5,0.5)
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(0),
                pose=Pose(MarkerPosition, MarkerQuaternion),
                scale=MarkerScale,
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), # green
                text=text)
        self.marker_publisher.publish(marker)
    
    def show_path_in_rviz(self,path):
        # path is a list of points that the robot pass
        #marker.show_path_in_rviz(path)
        MarkerPosition = Point(1,1,1.45)
        MarkerQuaternion = Quaternion(0,0,0,1)
        MarkerScale = Vector3(0.1,0.1,0.1)
        marker = Marker(
                type=Marker.LINE_STRIP,
                id=0,
                lifetime=rospy.Duration(0),
                pose=Pose(Point(), MarkerQuaternion),
                scale=MarkerScale,
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), # green
                points=path)
        self.marker_publisher.publish(marker)
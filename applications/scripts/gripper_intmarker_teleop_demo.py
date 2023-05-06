#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   gripper_intmarker_teleop_demo.py
@Time    :   2023/05/05 19:57:19
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

# here put the import lib
import rospy
import robot_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback

class GripperTeleop():
    """The Gripper Teleop 
    """
    def __init__(self,server_topic="gripper_im_server"):
        self._im_server = InteractiveMarkerServer(server_topic)
        self._fixed_frame = "odom"
        # set up pose listener
        self._poselistener = tf.TransformListener()
        rospy.sleep(0.5)
        
        # initialize the interactive marker 
        gripper_pose = self._getCurrentGripperPose()
        _IntMarkerCreater = robot_api.GripperInteractiveMarker()
        if gripper_pose is not None:
            self._IntMarker = _IntMarkerCreater.Create6DofMarker(gripper_pose)
            self._im_server.insert(self._IntMarker,self._HandleRvizCallback)
            self._im_server.applyChanges()
    def _getCurrentGripperPose(self,gripper_frame='/wrist_roll_link'):
        """Return the current gripper pose

        Args:
            gripper_frame (str, optional): The gripper frame. Defaults to '/wrist_roll_link'.

        Returns:
            gripper pose refer to self._fixed_frame
        """

        try:
            gripperPose = PoseStamped()
            gripperPose.header.frame_id = self._fixed_frame
            gripperPose.header.stamp = rospy.Time.now()
            (pose, rot) = self._poselistener.lookupTransform(gripper_frame,self._fixed_frame,rospy.Time(0))
            gripperPose.pose.position.x = pose[0]
            gripperPose.pose.position.y = pose[1]
            gripperPose.pose.position.z = pose[2]
            
            gripperPose.pose.orientation.x = rot[0]
            gripperPose.pose.orientation.y = rot[1]
            gripperPose.pose.orientation.z = rot[2]
            gripperPose.pose.orientation.w = rot[3]

            print(gripperPose)
            return gripperPose
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Transform error: {}".format(e))
            return None

    def _HandleRvizCallback(self,input):
        """The callback function

        Args:
            input (Rviz Input)
        """
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            pose = input.pose
            rospy.loginfo(pose)
    

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('Gripper_intmarker_teleop_demo')
    wait_for_time()
    Marker = GripperTeleop()
    rospy.sleep(0.5)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
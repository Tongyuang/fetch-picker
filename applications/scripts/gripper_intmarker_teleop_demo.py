#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   gripper_intmarker_teleop_demo.py
@Time    :   2023/05/05 19:57:19
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

import robot_api
# here put the import lib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from std_msgs.msg import ColorRGBA

class GripperTeleop():
    """The Gripper Teleop 
    """
    def __init__(self,server_topic="gripper_im_server", fixed_frame='base_link'):
        
        self._im_server = InteractiveMarkerServer(server_topic)
        self._fixed_frame = fixed_frame
        # set up pose listener
        self._poselistener = tf.TransformListener()
        rospy.sleep(0.5)
        
        # arm controller
        self.arm = robot_api.Arm()
        
        # gripper controller
        self.gripper = robot_api.Gripper()
        # initialize the interactive marker 
        gripper_pose = self._getCurrentGripperPose()
        self._IntMarkerCreator = robot_api.GripperInteractiveMarker(gripper_pose)
        self._im_server.insert(self._IntMarkerCreator.GetMarker(),self._HandleRvizCallback)
        self._ApplyMenuSetting() # create a self._menu_handler here
        self._im_server.applyChanges()

            
    def  _ApplyMenuSetting(self):
        """Apply the menu to the interactive marker
        """
        IntMarker = self._IntMarkerCreator.GetMarker()
        self._menulist = ["Move gripper here","Open Gripper", "Close Gripper"] # the order is important
        self._menu_handler = MenuHandler()
        for manu_name in self._menulist: 
            self._menu_handler.insert(manu_name,callback=self._HandleRvizCallback)
        # Apply the menu to the interactive marker
        self._menu_handler.apply(self._im_server, IntMarker.name)

        
        
    def _getCurrentGripperPose(self,gripper_frame='/wrist_roll_link'):
        """Return the current gripper pose

        Args:
            gripper_frame (str, optional): The gripper frame. Defaults to '/wrist_roll_link'.

        Returns:
            gripper pose refer to self._fixed_frame
        """
        gripperPose = PoseStamped()
        gripperPose.header.frame_id = self._fixed_frame
        try:

            #gripperPose.header.stamp = rospy.Time.now()
            (pose, rot) = self._poselistener.lookupTransform(self._fixed_frame,gripper_frame,rospy.Time(0))
            #gripperPose.header.stamp = rospy.Time.now()
            gripperPose.pose.position.x = pose[0]
            gripperPose.pose.position.y = pose[1]
            gripperPose.pose.position.z = pose[2]
            
            gripperPose.pose.orientation.x = rot[0]
            gripperPose.pose.orientation.y = rot[1]
            gripperPose.pose.orientation.z = rot[2]
            gripperPose.pose.orientation.w = rot[3]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Transform error: {}".format(e))
            rospy.logwarn("Cannot locate current gripper pose, gripper will be created at origin point")
            gripperPose.pose.orientation.w = 1
        return gripperPose

    # change color
    def _ChangeMarkerColor(self,colorname="red"):
        """Change the interactive marker's color

        Args:
            colorname (str, optional): colorname, either red or green so far. Defaults to "red".
        """
        
        if colorname == "red":
            color = ColorRGBA(r=0.9,g=0.1,b=0.0,a=1.0)
        elif colorname == "green":
            color = ColorRGBA(r=0.1,g=0.9,b=0.0,a=1.0)
        
        # change marker color
        self._IntMarkerCreator.UpdateColor(color)
        self._im_server.insert(self._IntMarkerCreator.GetMarker(),self._HandleRvizCallback)
        self._im_server.applyChanges()
        
    def _HandleRvizCallback(self,input):
        """The callback function

        Args:
            input (Rviz Input)
        """
        newpose = input.pose
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id = self._fixed_frame
        gripper_pose.pose = newpose
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP:

            # create a new marker
            self._IntMarkerCreator.Update(posestamped = gripper_pose)
            self._IntMarker = self._IntMarkerCreator.GetMarker()
            self._im_server.insert(self._IntMarker,self._HandleRvizCallback)
            self._ApplyMenuSetting()
            self._im_server.applyChanges()
            
            # update color
            if self.arm.compute_ik(gripper_pose):
                self._ChangeMarkerColor(colorname = "green")
            else:
                self._ChangeMarkerColor(colorname = "red")
            
        elif input.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            
            if self._menu_handler is not None:
                rospy.loginfo("Menu item {} selected".format(self._menulist[int(input.menu_entry_id)-1]))
                if input.menu_entry_id == 1: # Move gripper here
                    error = self.arm.move_to_pose(gripper_pose)
                    if error is not None:
                        rospy.logerr(error)
                    rospy.sleep(0.5)
                elif input.menu_entry_id == 2: # Open Gripper
                    self.gripper.open()
                    # to do: update the marker
                
                elif input.menu_entry_id == 3: # Close Gripper
                    effort = self.gripper.MAX_EFFORT
                    self.gripper.close(effort)
                    # to do : update the marker

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
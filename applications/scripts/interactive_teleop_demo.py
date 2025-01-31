#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File    :   interactive_teleop_demo.py
@Time    :   2023/04/24 14:33:28
@Author  :   Yuang Tong 
@Contact :   yuangtong1999@gmail.com
'''

# here put the import lib

import rospy
import robot_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point,PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry

import copy
import math
import tf.transformations as tft


class MyInteractiveMarker():
    def __init__(self):
        #self.server = InteractiveMarkerServer("simple_marker")
        self.server = InteractiveMarkerServer("/map_annotator/map_poses")
        # markers' distance to the robot
        self._markerDistance = 1
        self._robotPrevPosition = Odometry().pose.pose
        self._markerlist = []
        self.fixed_frame = "odom" # map or odom
        # create some interactive markers
        marker1 = self.CreateInteractiveMarker(position = Point(self._markerDistance,0,0.6),
                                               color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                               name = 'cubeforward',
                                               description = 'forward')
        self._markerlist.append(marker1)
        marker2 = self.CreateInteractiveMarker(position = Point(0,self._markerDistance,0.6),
                                               color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                               name = 'cubeleft',
                                               description = 'left')
        self._markerlist.append(marker2)
        marker3 = self.CreateInteractiveMarker(position = Point(-self._markerDistance,0,0.6),
                                               color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                               name = 'cubebackward',
                                               description = 'backward')
        self._markerlist.append(marker3)
        marker4 = self.CreateInteractiveMarker(position = Point(0,-self._markerDistance,0.6),
                                               color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                               name = 'cuberight',
                                               description = 'right')
        self._markerlist.append(marker4)
        
        for marker in self._markerlist:
            self.server.insert(marker,self.HandleRvizInput)
        #posestamped = PoseStamped()
        #posestamped.header.frame_id = self.fixed_frame
        #posestamped.header.stamp = rospy.Time.now()
        #posestamped.pose.orientation.w = 1
        
        #self.gripperinteractivemarker = robot_api.GripperInteractiveMarker(posestamped)
        #self.server.insert(self.gripperinteractivemarker.GetInteractiveMarker(),self.HandleRvizInput)
        #self.server.setCallback(self.gripperinteractivemarker.GetInteractiveMarker().name,self.HandleRvizInput)
        self.server.applyChanges()
        if self.fixed_frame == "odom":
            self.sub = rospy.Subscriber("odom", Odometry, self.UpdateMarkerPosition)
        elif self.fixed_frame == "map":
            self.sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.UpdateMarkerPosition)
        # create a driver
        self.driver = robot_api.Base()
    
    def CreateInteractiveMarker(self,
                                position = Point(x=1, y=0, z=0),
                                color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                name = 'simple cube button', 
                                description='my simple control'):
        '''Create an interactive button
            param color: ColorRGBA from std msg
            param position: Point from geometry_msgs.msg
            param name: Name of the cube
        '''
        
        # setup the BoxMarker(The CUBE)
        BoxMarker = Marker()
        BoxMarker.type = Marker.CUBE
        # set orientation
        BoxMarker.pose.orientation.w = 1
        # set scale, by default 0.45, 0.45, 0.45
        BoxMarker.scale.x = 0.45
        BoxMarker.scale.y = 0.45
        BoxMarker.scale.z = 0.45
        # set color
        BoxMarker.color= color
        
        # setup ButtonControl to control the cube
        ButtonControl = InteractiveMarkerControl()
        ButtonControl.interaction_mode = InteractiveMarkerControl.BUTTON
        ButtonControl.always_visible = True
        # add a box marker
        ButtonControl.markers.append(BoxMarker)
        # setup Interactive Marker for this button
        ThisInteractiveMarker = InteractiveMarker()
        ThisInteractiveMarker.header.frame_id = self.fixed_frame
        ThisInteractiveMarker.header.stamp = rospy.Time.now()
        ThisInteractiveMarker.name = name
        ThisInteractiveMarker.description = description
        # set orientation
        ThisInteractiveMarker.pose.orientation.w = 1
        # set position from input
        ThisInteractiveMarker.pose.position = position
        # ADD the controller
        ThisInteractiveMarker.controls.append(ButtonControl)
        
        return ThisInteractiveMarker

    def UpdateMarkerPosition(self,msg):
        '''This function subscribes the either /odom(for fixed_frame=odom) or /amcl(map) 
            topic and update the markers' position
            It will work as a callback function
        '''
        now_pos = msg.pose.pose
        robot_position = now_pos.position
        robot_orientation = now_pos.orientation

        _r = self._cal_radian(self._robotPrevPosition.orientation) - self._cal_radian(robot_orientation)
        

        # new orientation equals robot orientation
        # update marker positions
        for marker in self._markerlist:
            marker.pose.position = self._cal_new_pos(marker.pose.position,
                                                     self._robotPrevPosition.position,
                                                     robot_position,
                                                     _r)
            #print(marker.pose.position)
            marker.pose.orientation = robot_orientation
            #self.server.insert(marker,self.HandleRvizInput)
            
            
            self.server.setPose(marker.name, marker.pose)
        # apply changes
        self.server.applyChanges()
        # update position
        self._robotPrevPosition = copy.deepcopy(now_pos)
       
    def HandleRvizInput(self,input):
        # handles rviz input
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            # switch marker name
            if input.marker_name == 'cubeforward':
                self.driver.go_forward(0.5)
            elif input.marker_name == 'cubeleft':
                self.driver.turn(0.1)
            elif input.marker_name == 'cubebackward':
                self.driver.go_forward(-0.5)
            elif input.marker_name == 'cuberight':
                self.driver.turn(-0.1)               
            rospy.loginfo(input.marker_name + ' was clicked.')
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')
            
    def _cal_radian(self,quat):
        # calculate the radian given a quat
        mat = tft.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        return math.atan2(mat[0,0], mat[1,0])

    def _cal_new_pos(self,marker_origin,robot_origin, robot_new,gamma):
        new_marker_pos = copy.deepcopy(marker_origin)
        # update the vector after moving
        new_marker_pos.x += robot_new.x - robot_origin.x
        new_marker_pos.y += robot_new.y - robot_origin.y
        # calculate delta gamma
        rotate_vector = Point()
        rotate_vector.x = robot_new.x - new_marker_pos.x 
        rotate_vector.y = robot_new.y - new_marker_pos.y
        # rotate_vector rotates gamma
        new_rotate_vector = Point()
        new_rotate_vector.x = math.cos(gamma)*rotate_vector.x - math.sin(gamma)*rotate_vector.y
        new_rotate_vector.y = math.sin(gamma)*rotate_vector.x + math.cos(gamma)*rotate_vector.y
        # update the vector after rotating
        new_marker_pos.x = robot_new.x - new_rotate_vector.x
        new_marker_pos.y = robot_new.y - new_rotate_vector.y
        
        
        return new_marker_pos


def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('Interactive_Marker_demo')
    wait_for_time()
    Marker = MyInteractiveMarker()
    rospy.sleep(0.5)
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
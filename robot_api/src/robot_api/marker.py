#!/usr/bin/env python3

import rospy
import copy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback

import tf.transformations as tft
import numpy as np

class RvizMarker():
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

class GripperInteractiveMarker():
    """Create a gripper marker and return it
    """
    def __init__(self):
        self.mesh_sources = {
            'L_GRIPPER_MESH': 'package://fetch_description/meshes/l_gripper_finger_link.STL',
            'R_GRIPPER_MESH':'package://fetch_description/meshes/r_gripper_finger_link.STL',
            'GRIPPER_MESH':'package://fetch_description/meshes/gripper_link.dae'   
        }
    
    @staticmethod
    def _calWorldPos(posetarget,pose_refer_to_target):
        """Calculate position in world coordinate instead of refer to the 
            Pose target
        Args:
            posetarget (Pose): The target pose, say, (1,2,3) is the center of gripper
            pose_refer_to_target (Pose): the pose refer to the posetarget
        """
        pose_out = Pose()
        # get the posetarget transfer matrix
        quaternion_list = [posetarget.orientation.x,
                           posetarget.orientation.y,
                           posetarget.orientation.z,
                           posetarget.orientation.w]
        m = tft.quaternion_matrix(np.asarray(quaternion_list))
        m[0,-1] = posetarget.position.x
        m[1,-1] = posetarget.position.y
        m[2,-1] = posetarget.position.z

        # get the object  matrix refer to the target
        pose_refer_quaternion_list = [pose_refer_to_target.orientation.x,
                           pose_refer_to_target.orientation.y,
                           pose_refer_to_target.orientation.z,
                           pose_refer_to_target.orientation.w]
        mt = tft.quaternion_matrix(np.asarray(pose_refer_quaternion_list))
        mt[0,-1] = pose_refer_to_target.position.x
        mt[1,-1] = pose_refer_to_target.position.y
        mt[2,-1] = pose_refer_to_target.position.z       
        
        # calculate the object pose matrix in real world, ATC = ATB * BTC
        mt_world = np.dot(m,mt)
        
        # transfer back to pose in real world
        pose_out.position.x = mt_world[0,-1]
        pose_out.position.y = mt_world[1,-1]
        pose_out.position.z = mt_world[2,-1]
    
        quaternion_out = tft.quaternion_from_matrix(mt_world)
        pose_out.orientation.x = quaternion_out[0]
        pose_out.orientation.y = quaternion_out[1]
        pose_out.orientation.z = quaternion_out[2]
        pose_out.orientation.w = quaternion_out[3]
        
        return pose_out
        
    def CreateGripperMarkers(self,posestamped,frame_id='base_link'):
        marker_color = ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0)
        
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = self.mesh_sources['GRIPPER_MESH']
        gripper_marker.color = marker_color
        gripper_marker.pose = posestamped.pose
        gripper_marker.header.frame_id = frame_id
        
        l_gripper_marker = Marker()
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = self.mesh_sources['L_GRIPPER_MESH']        
        l_gripper_marker.color = marker_color
        
        l_refer_pose = Pose()
        l_refer_pose.orientation.w = 1
        l_refer_pose.position.y = -0.05
        l_gripper_marker.pose = self._calWorldPos(posestamped.pose,l_refer_pose)
        # l_gripper_marker.pose.position.y = -0.05 # for the gripper's correct pos
        # l_gripper_marker.pose.orientation.w = 1
        l_gripper_marker.header.frame_id = frame_id
        
        r_gripper_marker = Marker()
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = self.mesh_sources['R_GRIPPER_MESH']   
        r_gripper_marker.color = marker_color 
        
        r_refer_pose = Pose()
        r_refer_pose.orientation.w = 1
        r_refer_pose.position.y = 0.05
        r_gripper_marker.pose = self._calWorldPos(posestamped.pose,r_refer_pose)
        # r_gripper_marker.pose.position.y = 0.05
        # r_gripper_marker.pose.orientation.w = 1
        r_gripper_marker.header.frame_id = frame_id
        
        return {'gripper_marker':gripper_marker,
                'l_gripper_marker':l_gripper_marker,
                'r_gripper_marker':r_gripper_marker}
    
    def Create6DofMarker(self,posestamped,X_OFFSET=0.15):
        """Create a 6Dof Interactive Marker

        Args:
            posestamped (PoseStamped): The pose for the interactive marker
            X_OFFSET (float): we actually want the marker to be centered on the wrist_roll_link
        """
        
        # Create an interactive Marker
        intMarker = InteractiveMarker()
        intMarker.header.frame_id = posestamped.header.frame_id
        intMarker.scale = 0.5
        intMarker.name = "gripper_marker-6dof"
        intMarker.description = "gripper_marker-6dof"
        # initialize frame and position based on posestamped
        intMarker.header.stamp = posestamped.header.stamp
        intMarker.pose.position = posestamped.pose.position
        intMarker.pose.orientation = posestamped.pose.orientation
        
        # add a offset for the markers
        target_pose = Pose()
        target_pose.orientation.w = 1
        target_pose.position.x = X_OFFSET
        posestamped.pose = self._calWorldPos(posestamped.pose,target_pose)
        
        self.markers = self.CreateGripperMarkers(posestamped,frame_id = posestamped.header.frame_id)
        
        # insert the markers
        maincontrol = InteractiveMarkerControl()
        maincontrol.always_visible = True
        for marker_name in self.markers.keys():
            maincontrol.markers.append(self.markers[marker_name])
        intMarker.controls.append(maincontrol)
        
        # create a 6DoF controller
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        intMarker.controls.append(copy.deepcopy(control))
        
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.name = "move_x"
        intMarker.controls.append(copy.deepcopy(control))
        
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        intMarker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        intMarker.controls.append(copy.deepcopy(control))
        
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        intMarker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        intMarker.controls.append(copy.deepcopy(control))
        
        return intMarker
    
    def GetMarker(self):
        return self._6DofinteractiveMarker
    
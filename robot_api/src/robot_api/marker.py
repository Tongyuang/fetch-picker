#!/usr/bin/env python3

import rospy
import copy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback




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

class GripperInteractiveMarker():
    """Create a gripper marker and return it
    """
    def __init__(self):
        self.mesh_sources = {
            'L_GRIPPER_MESH': 'package://fetch_description/meshes/l_gripper_finger_link.STL',
            'R_GRIPPER_MESH':'package://fetch_description/meshes/r_gripper_finger_link.STL',
            'GRIPPER_MESH':'package://fetch_description/meshes/gripper_link.dae'   
        }
        self.markers = self.CreateGripperMarkers()
    
    def CreateGripperMarkers(self,frame_id='odom'):
        marker_color = ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0)
        
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = self.mesh_sources['GRIPPER_MESH']
        gripper_marker.color = marker_color
        gripper_marker.pose.orientation.w = 1
        gripper_marker.header.frame_id = frame_id
        
        l_gripper_marker = Marker()
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = self.mesh_sources['L_GRIPPER_MESH']        
        l_gripper_marker.color = marker_color
        l_gripper_marker.pose.position.y = -0.05 # for the gripper's correct pos
        l_gripper_marker.pose.orientation.w = 1
        l_gripper_marker.header.frame_id = frame_id
        
        r_gripper_marker = Marker()
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = self.mesh_sources['R_GRIPPER_MESH']   
        r_gripper_marker.color = marker_color 
        r_gripper_marker.pose.position.y = 0.05
        r_gripper_marker.pose.orientation.w = 1
        r_gripper_marker.header.frame_id = frame_id
        
        return {'gripper_marker':gripper_marker,
                'l_gripper_marker':l_gripper_marker,
                'r_gripper_marker':r_gripper_marker}
        
    def Create6DofMarker(self,posestamped):
        """Create a 6Dof Interactive Marker

        Args:
            posestamped (PoseStamped): The pose for the interactive marker
        """
        
        # Create an interactive Marker
        intMarker = InteractiveMarker()
        intMarker.header.frame_id = "odom"
        intMarker.scale = 1
        intMarker.name = "gripper_marker-6dof"
        
        # create a 6DoF controller
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.name = "move_x"
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        for marker in self.markers.keys():
            control.markers.append(self.markers[marker])
        intMarker.controls.append(copy.deepcopy(control))
        
        intMarker.description = "gripper_marker-6dof"
        # initialize frame and position based on posestamped
        intMarker.header.frame_id = posestamped.header.frame_id
        intMarker.header.stamp = posestamped.header.stamp
        intMarker.pose.position = posestamped.pose.position
        intMarker.pose.orientation = posestamped.pose.orientation

        print(intMarker.controls[2])
        return intMarker
        
        
        
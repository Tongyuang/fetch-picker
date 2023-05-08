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
    def __init__(self,posestamped):
        self.mesh_sources = {
            'L_GRIPPER_MESH': 'package://fetch_description/meshes/l_gripper_finger_link.STL',
            'R_GRIPPER_MESH':'package://fetch_description/meshes/r_gripper_finger_link.STL',
            'GRIPPER_MESH':'package://fetch_description/meshes/gripper_link.dae'   
        }
        self._grippercolor = ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0)   
        self.X_OFFSET = 0.177
        # create markers with offset
        target_pose = Pose()
        target_pose.orientation.w = 1
        target_pose.position.x = self.X_OFFSET
        markerPoseStamped = copy.deepcopy(posestamped)
        markerPoseStamped.pose = self._calWorldPos(posestamped.pose,target_pose)
        self.markers = self.CreateGripperMarkers(posestamped = markerPoseStamped, color = self._grippercolor)
        self._6DofinteractiveMarker = self.Create6DofMarker(self.markers,posestamped)
        
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
        
    def CreateGripperMarkers(self,posestamped,color=None):
        """Create Gripper Markers, which is in the shape of gripper. The __init__ function describes where the markers'
            mesh resources are located.

        Args:
            posestamped (PoseStamped): The position of the gripper
            marker_color (ColorRGBA, optional): The color of the markers. Defaults to ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0).

        Returns:
            dict, \{name: marker\}
        """
        frame_id = posestamped.header.frame_id
        marker_color = self._grippercolor if color is None else color
        
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = self.mesh_sources['GRIPPER_MESH']
        gripper_marker.color = marker_color
        gripper_marker.pose = posestamped.pose
        gripper_marker.header.frame_id = frame_id
        gripper_marker.text = 'gripper_marker'
        
        l_gripper_marker = Marker()
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = self.mesh_sources['L_GRIPPER_MESH']        
        l_gripper_marker.color = marker_color
        
        l_refer_pose = Pose()
        l_refer_pose.orientation.w = 1
        l_refer_pose.position.y = -0.05
        l_gripper_marker.pose = self._calWorldPos(posestamped.pose,l_refer_pose)
        l_gripper_marker.header.frame_id = frame_id
        l_gripper_marker.text = 'l_gripper_marker'
        
        r_gripper_marker = Marker()
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = self.mesh_sources['R_GRIPPER_MESH']   
        r_gripper_marker.color = marker_color 
        
        r_refer_pose = Pose()
        r_refer_pose.orientation.w = 1
        r_refer_pose.position.y = 0.05
        r_gripper_marker.pose = self._calWorldPos(posestamped.pose,r_refer_pose)
        r_gripper_marker.header.frame_id = frame_id
        r_gripper_marker.text = 'r_gripper_marker'
        
        return {'gripper_marker':gripper_marker,
                'l_gripper_marker':l_gripper_marker,
                'r_gripper_marker':r_gripper_marker}

    def Create6DofMarker(self,marker_dict,posestamped,intMarker_name="gripper_marker-6dof",marker_scale=0.5):
        """Create a 6 Dof Marker

        Args:
            posestamped (PoseStamped): The pose of the marker
            marker_dict (dict): the marker dict, as created in self.CreateGripperMarkers
            intMarker_name (str, optional): intMarker_name. Defaults to "gripper_marker-6dof".
            marker_scale (int, optional): the scale of the marker. Defaults to 0.5
        Returns:
            _type_: _description_
        """
        
        
        # Create an interactive Marker
        intMarker = InteractiveMarker()
        intMarker.header.frame_id = posestamped.header.frame_id
        intMarker.scale = marker_scale
        intMarker.name = intMarker_name
        intMarker.description = intMarker_name
        # initialize frame and position based on posestamped
        intMarker.header.stamp = posestamped.header.stamp
        intMarker.pose.position = posestamped.pose.position
        intMarker.pose.orientation = posestamped.pose.orientation
        
        # add a offset for the markers
        # target_pose = Pose()
        # target_pose.orientation.w = 1
        # target_pose.position.x = X_OFFSET
        # posestamped.pose = self._calWorldPos(posestamped.pose,target_pose)

        # self.CreateGripperMarkers(posestamped,color)
        # add a menu
        menucontrol = InteractiveMarkerControl()
        menucontrol.interaction_mode = InteractiveMarkerControl.MENU
        menucontrol.always_visible = True
        for marker_name in marker_dict.keys():
            menucontrol.markers.append(marker_dict[marker_name])
        intMarker.controls.append(menucontrol)
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
    
    def GetMarkerColor(self):
        return self._grippercolor
    
    def UpdateColor(self, color):
        """Update Markers' color

        Args:
            color (ColorRGBA): The target color
        """
        self._grippercolor = color
        for control in self._6DofinteractiveMarker.controls:
            for marker in control.markers:
                marker.color = color
                
    def Update(self,posestamped = None, color = None):
        
        if posestamped is not None:
            target_pose = Pose()
            target_pose.orientation.w = 1
            target_pose.position.x = self.X_OFFSET
            markerPoseStamped = copy.deepcopy(posestamped)
            markerPoseStamped.pose = self._calWorldPos(posestamped.pose,target_pose)
            self.markers = self.CreateGripperMarkers(posestamped = markerPoseStamped, color = self._grippercolor)
            self._6DofinteractiveMarker = self.Create6DofMarker(self.markers,posestamped)
        if color is not None:
            self.UpdateColor(color)


class GripperInteractiveMarkerWithObject(GripperInteractiveMarker):
    def __init__(self,posestamped):
        #super(self,GripperInteractiveMarker).__init__()
        #del GripperInteractiveMarker.__init__
        
        self._grippercolor = ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0)   
        self._cubecolor = ColorRGBA(r=0.9,g=0.9,b=0.1,a=1.0)   
        self.mesh_sources = {
            'L_GRIPPER_MESH': 'package://fetch_description/meshes/l_gripper_finger_link.STL',
            'R_GRIPPER_MESH':'package://fetch_description/meshes/r_gripper_finger_link.STL',
            'GRIPPER_MESH':'package://fetch_description/meshes/gripper_link.dae'   
        }
        self.GripperMarkerNameDict = {"pre-grasp": {'x':-0.1,'y':0.0,'z':0.0},
                                 "grasp": {'x':0.0,'y':0.0,'z':0.0},
                                 "lift": {'x':0.0,'y':0.0,'z':0.2}}
        self.Update(posestamped)
    
    def CreateCubeMarkers(self,posestamped,color=None):
        """Create a cube marker

        Args:
            posestamped (PoseStamped): The position of the gripper
            marker_color (ColorRGBA, optional): The color of the markers. Defaults to ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0).

        Returns:
            dict, \{name: marker\}
        """
        frame_id = posestamped.header.frame_id
        marker_color = self._cubecolor if color is None else color
        
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color = marker_color
        cube_marker.pose = posestamped.pose
        cube_marker.header.frame_id = frame_id
        cube_marker.text = 'gripper_marker'
        
        return {'cube_marker':cube_marker}
    
    def CreateOtherGripperMarkers(self,posestamped,color=None):
        """Create a Other Gripper Markers around the cube

        Args:
            posestamped (PoseStamped): The position of the gripper
            marker_color (ColorRGBA, optional): The color of the markers. Defaults to ColorRGBA(r=0.0,g=0.9,b=0.1,a=1.0).

        Returns:
            dict of dict, \{ name: \{name: marker\}\}
        """

        GripperMarkerDict = {}
        for name in self.GripperMarkerNameDict.keys():
            gripper_pose = copy.deepcopy(posestamped)
            target_pose = Pose()
            target_pose.orientation.w = 1
            target_pose.position.x = self.GripperMarkerNameDict[name]['x']
            target_pose.position.y = self.GripperMarkerNameDict[name]['y']
            target_pose.position.z = self.GripperMarkerNameDict[name]['z']
            gripper_pose.pose = self._calWorldPos(gripper_pose.pose,target_pose)
            GripperMarkerDict[name] = self.CreateGripperMarkers(copy.deepcopy(gripper_pose))
        
        return GripperMarkerDict
    
    def Update(self,posestamped = None, color = None):
            
        if posestamped is not None:
            self.markers = self.CreateCubeMarkers(posestamped = posestamped, color = self._cubecolor)
            self.gripper_markers = self.CreateOtherGripperMarkers(posestamped = posestamped, color = self._grippercolor)
            for gipper_class_name in self.gripper_markers.keys():
                for gripper_component_name in self.gripper_markers[gipper_class_name].keys():
                    self.markers[gipper_class_name+'_'+gripper_component_name] = self.gripper_markers[gipper_class_name][gripper_component_name]
                    self.markers[gipper_class_name+'_'+gripper_component_name].text = gipper_class_name+'_'+gripper_component_name
            self._6DofinteractiveMarker = self.Create6DofMarker(self.markers, posestamped, intMarker_name="cube_marker-6dof",marker_scale=0.1)
        if color is not None:
            self.UpdateColor(color)

    def GetGripperMarkerNameDict(self):
        return self.GripperMarkerNameDict
    
    def UpdateColorByName(self,name, color):
        """Update Markers' color

        Args:
            name: the marker name, in ["pre-grasp","grasp","lift"]
            color (ColorRGBA): The target color
        """
        assert name in self.GripperMarkerNameDict.keys()
        for control in self._6DofinteractiveMarker.controls:
            for marker in control.markers:
                if marker.text.startswith(name):
                    marker.color = color
        
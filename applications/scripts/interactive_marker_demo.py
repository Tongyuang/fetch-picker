#!/usr/bin/env python3

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class MyInteractiveMarker():
    def __init__(self):
        self.server = InteractiveMarkerServer("simple_marker")
        # create some interactive markers
        marker1 = self.CreateInteractiveMarker(position = Point(1,0,0),
                                               color = ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0),
                                               name = 'cube1')
        
        self.server.insert(marker1,self.HandleRvizInput)
        self.server.applyChanges()
    
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
        ThisInteractiveMarker.header.frame_id = "odom"
        ThisInteractiveMarker.name = name
        ThisInteractiveMarker.description = description
        # set orientation
        ThisInteractiveMarker.pose.orientation.w = 1
        # set position from input
        ThisInteractiveMarker.pose.position = position
        # ADD the controller
        ThisInteractiveMarker.controls.append(ButtonControl)
        
        return ThisInteractiveMarker

        
    def HandleRvizInput(self,input):
        # handles rviz input
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            rospy.loginfo(input.marker_name + ' was clicked.')
        else:
            rospy.loginfo('Cannot handle this InteractiveMarker event')

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('Interactive_Marker_demo')
    wait_for_time()
    Marker = MyInteractiveMarker()
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
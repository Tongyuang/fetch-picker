#!/usr/bin/env python

import rospy
import robot_api

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node('marker_demo')
    wait_for_time()
    marker = robot_api.RvizMarker()
    marker.show_text_in_rviz('I am robot Blitzcrank')
    
    
    
if __name__ == '__main__':
    main()
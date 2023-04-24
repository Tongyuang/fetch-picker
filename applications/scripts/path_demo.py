#!/usr/bin/env python3

import rospy
import robot_api

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass


def main():
    rospy.init_node('path_demo')
    wait_for_time()
    
    marker = robot_api.RvizMarker()
    path_tracker = robot_api.PosTracker()

    #path = path_tracker.GetPosList()
    #logstr = 'x:{},y:{},z:{}'.format(path[-1].x, path[-1].y, path[-1].z)
    #rospy.loginfo(logstr)
    
    #marker.show_path_in_rviz('path',path_tracker)
    #marker.show_text_in_rviz('I am robot Blitzcrank')
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
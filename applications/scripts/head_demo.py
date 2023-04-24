#! /usr/bin/env python

import rospy
import robot_api

INFO_MSG = '''

Usage:
    rosrun applications head_demo.py look_at FRAME_ID X Y Z
    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG
    rosrun applications head_demo.py eyes ANG
Examples:
    rosrun applications head_demo.py look_at base_link 1 0 0.3
    rosrun applications head_demo.py pan_tilt 0 0.707
    rosrun applications head_demo.py eyes .50
    
'''

def print_usage():
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    
    print(INFO_MSG)


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('head_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    head_controller = robot_api.Head()
    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id, x, y, z = argv[2], float(argv[3]), float(argv[4]), float(
            argv[5])
        
        head_controller.look_at(frame_id,x,y,z)
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan, tilt = float(argv[2]), float(argv[3])
        head_controller.pan_tilt(pan,tilt)
    elif command == 'eyes':
        if len(argv) < 3:
            print_usage()
            return
        angle = float(argv[2])
        head_controller.pan_tilt(tilt = angle)
    else:
        print_usage()


if __name__ == '__main__':
    main()

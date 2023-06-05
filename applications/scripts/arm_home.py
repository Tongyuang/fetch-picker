#! /usr/bin/env python3

import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    # #I change the name according to the error:
    rospy.init_node('arm_demo')
    # rospy.init_node('fetch_picker')
    wait_for_time()
    argv = rospy.myargv()
    DISCO_POSES = [[1.32, 1.40, -0.20, 1.72, 0.0, 1.66, 0.0],
                   ]

    torso = robot_api.Torso()
    torso.set_height(robot_api.Torso.MAX_HEIGHT)

    arm = robot_api.Arm()
    for vals in DISCO_POSES:
        arm.move_to_joints(robot_api.ArmJoints.from_list(vals))
    
    # torso.set_height(robot_api.Torso.MIN_HEIGHT)


if __name__ == '__main__':
    main()

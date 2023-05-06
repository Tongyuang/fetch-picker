#! /usr/bin/env python

import rospy
import tf



def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('fetch_tf_listener_demo')
    wait_for_time()
    
    listener = tf.TransformListener()
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        try:
            (l_gripper,rot) = listener.lookupTransform('/l_gripper_finger_link', '/odom', rospy.Time(0))
            rospy.loginfo("L gripper pose: {}, rotation:{}".format(l_gripper,rot))
            
            (r_gripper,rot) = listener.lookupTransform('/r_gripper_finger_link', '/odom', rospy.Time(0))
            rospy.loginfo("R gripper pose: {}, rotation:{}".format(r_gripper,rot))
            
            (gripper,rot) = listener.lookupTransform('/wrist_roll_link', '/odom', rospy.Time(0))
            rospy.loginfo("Gripper pose: {}, rotation:{}".format(gripper,rot))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue
            pass
        rate.sleep()
if __name__ == "__main__":
    main()
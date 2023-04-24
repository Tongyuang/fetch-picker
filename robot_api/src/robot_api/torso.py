#!/usr/bin/env python3

import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# TODO: ACTION_NAME = ???
ACTION_NAME = '/torso_controller/follow_joint_trajectory'
# TODO: JOINT_NAME = ???
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for action server...")
        # TODO: Wait for server
        self.client.wait_for_server()
        
        rospy.loginfo("Action server started.") 

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            rospy.logwarn("Height value must within {} - {}, but got {}. Robot not moving.".format(self.MIN_HEIGHT,self.MAX_HEIGHT, height))
            return False
        # TODO: Create a trajectory point
        traj = JointTrajectory()
        traj.joint_names = [JOINT_NAME]
        # TODO: Set position of trajectory point
        point = JointTrajectoryPoint()
        point.positions = [height]
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        # TODO: Add joint name to list
        traj.points.append(point)
        
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory = traj
        # TODO: Send goal
        self.client.send_goal(goal)
        
        # TODO: Wait for result
        self.client.wait_for_result()
        
        return self.client.get_result()

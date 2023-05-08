#! /usr/bin/env python3

# TODO: import ?????????
# TODO: import ???????_msgs.msg
import rospy
import control_msgs
import control_msgs.msg
import actionlib

from control_msgs.msg import GripperCommandGoal,GripperCommandAction

# TODO: ACTION_NAME = ???
ACTION_NAME = '/gripper_controller/gripper_action'
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, GripperCommandAction)
        rospy.loginfo("Waiting for action server...")
        # TODO: Wait for server
        self.client.wait_for_server()
        
        rospy.loginfo("Action server started.") 
        
        
        #pass

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        rospy.loginfo('Opening the gripper.')
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS
        # TODO: Send goal
        self.client.send_goal(goal)
        # TODO: Wait for result
        self.client.wait_for_result()

        return self.client.get_result

    def close(self, position=None,max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
            position: the close position, should range in [0-0.1], default None
        """
        # TODO: Create goal
        if position is not None:
            assert position < 0.1 and position >= 0
            close_pos = position
        else:
            close_pos = CLOSED_POS
        
        rospy.loginfo('Closing the gripper.')
        goal = GripperCommandGoal()
        goal.command.position = close_pos
        goal.command.max_effort = max_effort
        # TODO: Send goal
        self.client.send_goal(goal)
        # TODO: Wait for result
        self.client.wait_for_result()

        return self.client.get_result
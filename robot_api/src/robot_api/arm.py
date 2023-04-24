# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib

from .arm_joints import ArmJoints

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

ACTION_NAME = '/arm_controller/follow_joint_trajectory'
TIME_FROM_START = 5 

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """
    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        # TODO: Wait for server
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        
        rospy.loginfo("Action server started.") 

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()

        # TODO: Set position of trajectory point
        point.positions = arm_joints.values()
        
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        
        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        
        # TODO: Add joint name to list
        goal.trajectory.joint_names = arm_joints.names()
        
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)
        
        # TODO: Send goal
        self.client.send_goal(goal)
        
        # TODO: Wait for result
        
        self.client.wait_for_result()
        
        return self.client.get_result()

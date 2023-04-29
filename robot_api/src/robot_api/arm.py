import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder

from collections import defaultdict

ACTION_NAME = '/arm_controller/follow_joint_trajectory'
MOVE_GROUP_ACTION_NAME = 'move_group'
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
        self._client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        # create a movegroupaction client:
        self._move_group_client = actionlib.SimpleActionClient(MOVE_GROUP_ACTION_NAME,MoveGroupAction)
        # TODO: Wait for server
        rospy.loginfo("Waiting for action server...")
        self._client.wait_for_server()
        self._move_group_client.wait_for_server()
        
        rospy.loginfo("Action server started.") 
        # create error dict
        self._create_error_dict()

    def _create_error_dict(self):
        """create an error dict for MoveItErrorCode
        """
        self._MoveItErrorCodeDict = defaultdict()
        for attr_name in dir(MoveItErrorCodes):
            attr_val = getattr(MoveItErrorCodes, attr_name)
            if isinstance(attr_val, int) and (not attr_name.startswith('_')):
                self._MoveItErrorCodeDict[attr_val] = attr_name

    
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
        self._client.send_goal(goal)
        
        # TODO: Wait for result
        
        self._client.wait_for_result()
        
        return self.client.get_result()

    def move_to_pose(self,pose_stamped):
        """Moves the robot's arm to the given pose.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal = goal_builder.build() # a MoveGroupGoal
        # try to move to the goal
        self._move_group_client.send_goal(goal)
        self._move_group_client.wait_for_result(rospy.Duration(10))
        val = self._move_group_client.get_result().error_code.val
        if val == MoveItErrorCodes.SUCCESS:
            return None
        return self.moveit_error_string(val)
    
    def cancel_all_goals(self):
        self._client.cancel_all_goals() 
        self._move_group_client.cancel_all_goals() 
        
    def moveit_error_string(self,val):
        """Returns a string associated with a MoveItErrorCode.
        
        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 
        if val in self._MoveItErrorCodeDict.keys():
            return self._MoveItErrorCodeDict[val]
        else:
            return 'UNKNOWN_ERROR_CODE'

from collections import defaultdict

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder



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
        rospy.loginfo("Action server started.") 
        # rospy.loginfo("Waiting for move group server...")
        # self._move_group_client.wait_for_server()
        # rospy.loginfo("move group server started.") 
        
        # create error dict
        self._create_error_dict()
        
        # for inverse kinetics
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

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
        
        return self._client.get_result()

    def move_to_pose(self,
                    pose_stamped,
                    orientation_constraint=None,
                    allowed_planning_time=10.0,
                    execution_timeout=15.0,
                    group_name='arm',
                    num_planning_attempts=1,
                    plan_only=False,
                    replan=False,
                    replan_attempts=5,
                    tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.group_name = group_name
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
                # add orientation_constraint
        if not orientation_constraint is None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)
        
        goal = goal_builder.build() # a MoveGroupGoal
        
        # try to move to the goal
        self._move_group_client.send_goal(goal)
        self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        
        # get the error code val
        result = self._move_group_client.get_result()
        if result is not None:
            val = result.error_code.val
        
            if val == MoveItErrorCodes.SUCCESS:
                return None
            else:
                return self.moveit_error_string(val)
        else:
            return "No result from move_group_client"
    
    def check_pose(self,pose_stamped,allowed_planning_time=10.0,group_name='arm',tolerance=0.01):
        """_summary_

        Args:
            pose_stamped geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time (float, optional): The maximum duration to wait for a
                planning result, in seconds. Defaults to 10.0.
            group_name (str, optional): Either 'arm' or 'arm_with_torso'. Defaults to 'arm'.
            tolerance (float, optional): float. The goal tolerance, in meters. Defaults to 0.01.
        """
        assert group_name in ['arm','arm_with_torso']
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time = allowed_planning_time,
            group_name = group_name,
            tolerance = tolerance,
            plan_only = True
        )
    
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = self.moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True
    
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

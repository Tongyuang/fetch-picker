#! /usr/bin/env python3

# TODO: import ????????_msgs.msg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import copy
import math
import tf.transformations as tft

class Base():
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        # pass
        self._odomSub = rospy.Subscriber('odom',Odometry, callback=self._odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._CurPos = None
        
    def _odom_callback(self, msg):
        # get the current position
        position = msg.pose.pose.position
        self._CurPos = msg.pose.pose
    
    def _cal_distance(self,pos1,pos2):
        # calculate the Eucilidean distance of two positions
        return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2 + (pos1.z-pos2.z)**2)
    
    def _cal_radian(self,quat):
        # calculate the radian given a quat
        mat = tft.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        return math.atan2(mat[0,0], mat[1,0])

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
            means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        rate = rospy.Rate(10)
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while(self._CurPos is None):
            rate.sleep()
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._CurPos)
        
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        while True:
            now_pos = self._CurPos
            dist = self._cal_distance(start.position,now_pos.position)
            if dist >= abs(distance):
                break
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
    
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
        angular_distance: The angle, in radians, to rotate. A positive
        value rotates counter-clockwise.
        speed: The angular speed to rotate, in radians/second.
        """
        rate = rospy.Rate(10)
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while(self._CurPos is None):
            rate.sleep()
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._CurPos)
        start_rad = self._cal_radian(start.orientation)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        angular_distance = math.atan2(math.sin(angular_distance),math.cos(angular_distance))
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while True:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            now_pos = self._CurPos
            now_rad = self._cal_radian(now_pos.orientation)
            if abs(now_rad - start_rad) >= abs(angular_distance):
                break
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()    
        
            
    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        
        # TODO: Create Twist msg
        twist_msg = Twist()
        
        # TODO: Fill out msg
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        # TODO: Publish msg
        self.pub.publish(twist_msg)
        #rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        self.move(0,0)
        #rospy.logerr('Not implemented.')

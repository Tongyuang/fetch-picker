import rospy
import robot_api

from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry

class PosTracker():
    '''
    Tracks the robot's position
    '''
    
    def __init__(self):
        # subscriber
        self._PosList = []
        self.subscriber = rospy.Subscriber('odom', Odometry, self.PosTrackerCallback)
        rospy.sleep(0.5)
        # for visualizatoin
        self.marker = robot_api.RvizMarker()
        
    def PosTrackerCallback(self,msg):
        # get the current position
        position = msg.pose.pose.position
        # log the info of position
        logstr = 'x:{},y:{},z:{}'.format(position.x, position.y, position.z)
        rospy.loginfo(logstr)
        # detect if the position can be added to path
        if self.DetectMovement(position):
            self._PosList.append(position)
            rospy.loginfo('Path Updated!')
            # print pos list when needed
            if len(self.GetPosList())>1:
                self.marker.show_path_in_rviz(self.GetPosList())
        
    def DetectMovement(self,position,threshold = 0.01):
        now_x,now_y,now_z = position.x, position.y, position.z
        # if empty, add by default
        if len(self.GetPosList())==0:
            return True
        # get the previous position
        prev_pos = self.GetPosList()[-1]
        prev_x,prev_y,prev_z = prev_pos.x,prev_pos.y,prev_pos.z
        # judge by Eucilidean Distance
        return ((now_x-prev_x)**2 + (now_y-prev_y)**2 + (now_z-prev_z)**2) > threshold
    
    def GetPosList(self):
        return self._PosList
        
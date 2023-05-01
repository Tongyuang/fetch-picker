#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft
import copy

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def cosd(degs):
    return math.cos(degs * math.pi / 180)


def sind(degs):
    return math.sin(degs * math.pi / 180)


def axis_marker(pose_stamped):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'axes'
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    marker.scale.x = 0.1

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))

    return marker


def transform_to_pose(matrix):
    pose = Pose()
    # TODO: fill this out
    pose.position.x = matrix[0,-1]
    pose.position.y = matrix[1,-1]
    pose.position.z = matrix[2,-1]
    
    quaternion = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    
    return pose

def transform_to_matrix(pose):
    quaternion_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    matrix = tft.quaternion_matrix(np.asarray(quaternion_list))
    matrix[0,-1] = pose.position.x
    matrix[1,-1] = pose.position.y
    matrix[2,-1] = pose.position.z
    return matrix

def arrow_marker(point):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'arrow'
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.header.frame_id = 'frame_a'
    marker.points.append(Point(0, 0, 0))
    marker.points.append(point)
    marker.scale.x = 0.1
    marker.scale.y = 0.15
    marker.color.r = 1
    marker.color.g = 1
    marker.color.a = 1
    return marker

def pre_grasp_demo():
    
    demo_object = Pose()
    demo_object.position.x = 0.6
    demo_object.position.y = -0.1
    demo_object.position.z = 0.7
    
    demo_object.orientation.x = 0
    demo_object.orientation.y = 0
    demo_object.orientation.z = 0.38268343
    demo_object.orientation.w = 0.92387953
    
    # to matrix4
    poseMatrix = transform_to_matrix(demo_object)
    
    expectedPoseFromObjectView = np.asarray([-0.1,0,0,1])
    poseFromBaseLink = np.dot(poseMatrix,expectedPoseFromObjectView)
    
    output_pose = copy.deepcopy(demo_object)
    output_pose.position.x = poseFromBaseLink[0]
    output_pose.position.y = poseFromBaseLink[1]
    output_pose.position.z = poseFromBaseLink[2]
    
    return output_pose
    

def main():
    rospy.init_node('transformation_demo')
    wait_for_time()
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)
    b_in_a = np.array([
        [cosd(45), -sind(45), 0, 0],
        [sind(45), cosd(45), 0, 0],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1]
    ])
    ps = PoseStamped()
    ps.header.frame_id = 'frame_a'
    ps.pose = transform_to_pose(b_in_a)
    viz_pub.publish(axis_marker(ps))

    point_in_b = np.array([1, 0, 0, 1])
    point_in_a = np.dot(b_in_a, point_in_b)
    rospy.loginfo(point_in_b)
    rospy.loginfo(point_in_a)
    point = Point(point_in_a[0], point_in_a[1], point_in_a[2])
    viz_pub.publish(arrow_marker(point))
    
if __name__ == '__main__':
    print(pre_grasp_demo())
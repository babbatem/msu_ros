#!/bin/bash

import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import pickle
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


from visualization_msgs.msg import Marker


from tf import transformations as tft

# init moveit
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasps_to_poses', anonymous=True)
robot = moveit_commander.RobotCommander(ns='j2s6s300')
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)


# waypoints = []
# scale=1.0
# wpose = group.get_current_pose().pose
# wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# (plan, fraction) = group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0.0)         # jump_threshold
# group.execute(plan, wait=True)
#

# rospy.loginfo('--')
# current_pose = group.get_current_pose().pose
# rospy.loginfo('target pos: %f %f %f' %(pose_goal.position.x,
#                                 pose_goal.position.y,
#                                 pose_goal.position.z))
#
# rospy.loginfo('current pos: %f %f %f' %(current_pose.position.x,
#                                       current_pose.position.y,
#                                       current_pose.position.z))

# print('target ori: %f %f %f %f' %(pose_goal.orientation.w,
#                                    pose_goal.orientation.x,
#                                    pose_goal.orientation.y,
#                                    pose_goal.orientation.z))
#
# print('current_pose ori: %f %f %f %f' %(current_pose.orientation.w,
#                                         current_pose.orientation.x,
#                                          current_pose.orientation.y,
#                                          current_pose.orientation.z))

def gpd_msg_to_quat(g):
    # extract rotation from 3 grasp axes
    a = g.approach
    b = g.binormal
    c = g.axis

    # TODO: generalize. multiply by gripper's orientation matrix.
    # rotmat = np.array([[a.x, b.x, c.x, 0],
    #                    [a.y, b.y, c.y, 0],
    #                    [a.z, b.z, c.z, 0],
    #                    [  0,   0,   0, 1]])

    #
    #
    rotmat = np.array([[b.x, c.x, a.x, 0],
                       [b.y, c.y, a.y, 0],
                       [b.z, c.z, a.z, 0],
                       [  0,   0,   0, 1]])  # IMPLICIT CONVERSION TO EE FRAME

    quat = tft.quaternion_from_matrix(rotmat)
    # print(quat)

    return quat



# load GPD grasps
grasps_path = '/home/abba/msu_ws/src/msu_ros/data/gpd_grasps_handle_only'
file = open(grasps_path,'rb')
grasps_list = pickle.load(file)
print(len(grasps_list))

import sys; sys.exit()

out=[]
# pose_goal=group.get_current_pose().pose

pub1 = rospy.Publisher('visualization_marker', Marker, queue_size=1)

# for each, use moveit to reach the pose
for i in range(len(grasps_list)):

    quat = gpd_msg_to_quat(grasps_list[i])

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = quat[-1]
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.position.x = grasps_list[i].position.x
    pose_goal.position.y = grasps_list[i].position.y
    pose_goal.position.z = grasps_list[i].position.z

    marker_ = Marker()
    marker_.header.frame_id = "/world"
    marker_.header.stamp = rospy.Time.now()
    marker_.type = marker_.ARROW
    marker_.pose=pose_goal
    marker_.lifetime = rospy.Duration.from_sec(1000)
    marker_.scale.x = 0.1
    marker_.scale.y = 0.03
    marker_.scale.z = 0.01
    marker_.color.a = 1
    red_, green_, blue_ = (1, 0, 0)
    marker_.color.r = red_
    marker_.color.g = green_
    marker_.color.b = blue_
    pub1.publish(marker_)

    group.set_pose_target(pose_goal)
    plan1 = group.plan()
    print('----')
    # print(plan1)
    print(len(plan1.joint_trajectory.points))
    print('----')
    # group.go(wait=True)

    # time.sleep(5.0)
    # group.stop()
    # group.clear_pose_targets()

    # record the resulting arm pose
    out.append(group.get_current_joint_values())

file2 = open('outie','wb')
pickle.dump(out, file2)

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander(ns='j2s6s300')

scene = moveit_commander.PlanningSceneInterface()


group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)


while True:
    # We can get the joint vales from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 1.0 + np.random.rand()*2.0
    joint_goal[1] = 1.0 + np.random.rand()*2.0
    joint_goal[2] = 1.0 + np.random.rand()*2.0
    joint_goal[3] = 1.0 + np.random.rand()*2.0
    joint_goal[4] = 1.0 + np.random.rand()*2.0
    joint_goal[5] = 1.0 + np.random.rand()*2.0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()``
    move_group.stop()

    print('target: ', joint_goal)
    print('current: ', move_group.get_current_joint_values())

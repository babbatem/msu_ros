
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



file = open('/home/abba/msu_ws/src/msu_ros/data/quats_latest','rb')
qs=pickle.load(file)

file = open('/home/abba/msu_ws/src/msu_ros/data/xyz_latest','rb')
xyz=pickle.load(file)
print(len(qs))
print(len(xyz))
out=[]

pub1 = rospy.Publisher('visualization_marker', Marker, queue_size=1)

keep = []

# for each, use moveit to reach the pose
for i in range(len(qs)):

    quat=qs[i]
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = quat[-1]
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.position.x = xyz[i][0]
    pose_goal.position.y = xyz[i][1]
    pose_goal.position.z = xyz[i][2]

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
    # group.go(wait=True)

    if len(plan1.joint_trajectory.points) > 0:
        print(plan1.joint_trajectory.points[-1].positions)
        out.append(plan1.joint_trajectory.points[-1].positions)
        keep.append(copy.deepcopy(i))

    # time.sleep(5.0)
    # group.stop()
    group.clear_pose_targets()

    # record the resulting arm pose

file2 = open('joint_angles_latest','wb')
pickle.dump(out, file2)

file3 = open('data_dict', 'wb')
pickle.dump( dict(xyz=np.array(xyz)[keep],
                  quat=np.array(qs)[keep],
                  joint_pos=np.array(out)),
             file3)

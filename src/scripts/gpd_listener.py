#!/usr/bin/env python
import rospy
from gpd_ros.msg import GraspConfigList, GraspConfig
from geometry_msgs.msg import Point, Vector3
from tf import transformations as tft
import pickle
import numpy as np

# # Position
# geometry_msgs/Point position # grasp position (bottom/base center of robot hand)
#
# # Orientation represented as three axis (R = [approach binormal axis])
# geometry_msgs/Vector3 approach # grasp approach direction
# geometry_msgs/Vector3 binormal # hand closing direction
# geometry_msgs/Vector3 axis # hand axis
#
# std_msgs/Float32 width # Required aperture (opening width) of the robot hand
#
# std_msgs/Float32 score # Score assigned to the grasp by the classifier
def callback(data):
    rospy.loginfo("received %i grasps " % (len(data.grasps)))
    file = open('/home/abba/msu_ws/src/msu_ros/data/gpd_grasps_latest','wb')
    pickle.dump(data.grasps, file)
    file.close()

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

class GPDListener(object):
    """docstring for GPDListener."""
    def __init__(self):
        super(GPDListener, self).__init__()
        self.quats = []
        self.xyzs = []
        self.msgs = []


    def callback2(self, data):
        rospy.loginfo("received %i grasps " % (len(data.grasps)))

        self.msgs.append(data.grasps)

        quats_t = [gpd_msg_to_quat(data.grasps[i]) for i in range(len(data.grasps))]
        for q in quats_t:
            self.quats.append(q)

        xs = [ g.position.x for g in data.grasps ]
        ys = [ g.position.y for g in data.grasps ]
        zs = [ g.position.z for g in data.grasps ]
        xyz_t = [ [xs[i], ys[i], zs[i]] for i in range(len(xs)) ]

        # haven't run this bit yet 
        for point in xyz_t:
            self.xyzs.append(point)

        file = open('/home/abba/msu_ws/src/msu_ros/data/quats_latest','wb')
        pickle.dump(self.quats, file)
        file = open('/home/abba/msu_ws/src/msu_ros/data/xyz_latest','wb')
        pickle.dump(self.xyzs, file)

        file.close()
        return

def listener():
    L = GPDListener()
    rospy.init_node('gpd_listener')
    rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, L.callback2)
    rospy.spin()

if __name__ == '__main__':
    listener()

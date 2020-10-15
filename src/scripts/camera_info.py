import numpy as np
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import rospy
import tf
from tf.transformations import quaternion_from_euler

class Camera:
    def __init__(self, camera_name, im_size, frame_id, pos, ori):
        self.camera_name = camera_name
        self.im_size = im_size

        # create Header
        self.header = Header()
        self.header.frame_id = frame_id

        # me being naive
        fovy = 45
        znear = 0.01
        zfar = 50

        cx = im_size[0] / 2.
        cy = im_size[1] / 2.
        fy = (im_size[1] / 2.) / np.tan(fovy*np.pi/180.0)
        fx = fy
        self.K = np.array([[fx,  0, cx],
                           [ 0, fy, cy],
                           [ 0,  0,  1]])

        self.R = np.eye(3, dtype=np.float64)
        self.P = self.K

        self.ci = CameraInfo()
        self.ci.header = self.header
        self.ci.height = self.im_size[1]
        self.ci.width = self.im_size[0]
        self.ci.K[:] = self.K.flatten()
        self.ci.R[:] = self.R.flatten()
        self.ci.P[:9] = self.P.flatten()

        self.caminfo_pub = rospy.Publisher(camera_name + "_info",
                                           CameraInfo,
                                           queue_size=5)


        self.pos = pos
        self.ori = ori
        self.tf_br = tf.TransformBroadcaster()

    def spin(self):
        self.caminfo_pub.publish(self.ci)
        self.tf_br.sendTransform(self.pos,
                                 quaternion_from_euler(self.ori[0],
                                                       self.ori[1],
                                                       self.ori[2]),
                                 rospy.Time.now(),
                                 self.camera_name,
                                 "world")

if __name__ == '__main__':
     rospy.init_node('single_cam_tf_broadcaster')

     camera_name = "cam_1"
     im_size = (192,108)
     frame_id = camera_name
     pos = (1.0, 0, 1.00)
     ori = (0, 0.753, 1.57)
     cam=Camera(camera_name, im_size, frame_id, pos, ori)


     r = rospy.Rate(30)
     while not rospy.is_shutdown():
         cam.spin()
         r.sleep()

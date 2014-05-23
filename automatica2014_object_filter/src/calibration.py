#!/usr/bin/python

import roslib
roslib.load_manifest('cob_fiducials')
import rospy
from tf import TransformListener
import tf

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *

'''
TF-Tree:

given:
frame_eye --> frame_camera_mount
frame_marker --> frame_marker_mount

wanted:
frame_marker_mount --> frame_camera_mount

fidicuials:
frame_marker --> frame_eye
'''

class calib:
    def __init__(self, *args):
        self.tf = TransformListener()
        self.detector_service = rospy.ServiceProxy("/fiducials/get_fiducials", DetectObjects)

        self.frame_camera_mount = rospy.get_param('~frame_camera_mount')
        self.frame_marker_mount = rospy.get_param('~frame_marker_mount')
        self.frame_marker = rospy.get_param('~frame_marker', "/marker")

    def compute(self):
        res = self.detector_service(DetectObjectsRequest())

        if self.tf.frameExists(self.frame_camera_mount) and self.tf.frameExists(self.frame_marker_mount):
            t = self.tf.getLatestCommonTime(self.frame_camera_mount, self.frame_marker_mount)
            position, quaternion = self.tf.lookupTransform(self.frame_camera_mount, self.frame_marker_mount, t)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            print '<origin xyz="'+str(position[0])+" "+str(position[1])+" "+str(position[2])+'" rpy="'+str(euler[0])+" "+str(euler[1])+" "+str(euler[2])+'" />'
        else:
            print "tf does not exist!", self.tf.frameExists(self.frame_camera_mount), self.tf.frameExists(self.frame_marker_mount)

if __name__ == '__main__':
	rospy.init_node('marker_transform')
	node = calib()
	node.compute()

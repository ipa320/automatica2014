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
        
        x_values = []
        y_values = []
        z_values = []
        roll_values = []
        pitch_values = []
        yaw_values = []
        
        count_success = 0
        count_failed = 0
        while count_success <= 15 and count_failed <= 100:
            try: 
                rospy.wait_for_service("/fiducials/get_fiducials", 3.0)
                res = self.detector_service(DetectObjectsRequest())
                
                if self.tf.frameExists(self.frame_camera_mount) and self.tf.frameExists(self.frame_marker_mount):
                    t = self.tf.getLatestCommonTime(self.frame_camera_mount, self.frame_marker_mount)
                    position, quaternion = self.tf.lookupTransform(self.frame_camera_mount, self.frame_marker_mount, t)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    print '<origin xyz="'+str(position[0])+" "+str(position[1])+" "+str(position[2])+'" rpy="'+str(euler[0])+" "+str(euler[1])+" "+str(euler[2])+'" />'
                    x_values.append(position[0])
                    y_values.append(position[1])
                    z_values.append(position[2])
                    roll_values.append(euler[0])
                    pitch_values.append(euler[1])
                    yaw_values.append(euler[2])
                else:
                    print "tf does not exist!", self.tf.frameExists(self.frame_camera_mount), self.tf.frameExists(self.frame_marker_mount)
            except:
                print "did not detect marker."
                print "count_success = ", count_success
                print "count_failed = ", count_failed
                count_failed += 1
                continue
            count_success += 1
        
        if len(x_values) < 5:
            print "to few detections, aborting"
            return
                
        x_avg_value = (float)(sum(x_values))/len(x_values)
        y_avg_value = (float)(sum(y_values))/len(y_values)
        z_avg_value = (float)(sum(z_values))/len(z_values)
        roll_avg_value = (float)(sum(roll_values))/len(roll_values)
        pitch_avg_value = (float)(sum(pitch_values))/len(pitch_values)
        yaw_avg_value = (float)(sum(yaw_values))/len(yaw_values)

        print "The average values (without hardcoding) <xyz> are : \n\n" + '<origin xyz="' + str(x_avg_value) +" "+ str(-y_avg_value) +" "+  str(z_avg_value) + '" rpy="' + str(roll_avg_value) + " " + str(pitch_avg_value) + " " + str(yaw_avg_value) + '" />\n'
        
        # HACK set some DOf to fixed values
        z_avg_value = 1.80
        roll_avg_value = -3.1415926
        pitch_avg_value = 0
        yaw_avg_value = -3.1415926
        
        print "The average values<xyz> are : \n\n" + '<origin xyz="' + str(x_avg_value) +" "+ str(-y_avg_value) +" "+  str(z_avg_value) + '" rpy="' + str(roll_avg_value) + " " + str(pitch_avg_value) + " " + str(yaw_avg_value) + '" />\n'

if __name__ == '__main__':
	rospy.init_node('marker_transform')
	node = calib()
	node.compute()

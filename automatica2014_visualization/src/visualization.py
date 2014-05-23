#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_visualization')

import sys

import rospy
import copy
from visualization_msgs.msg import *
from geometry_msgs.msg import *
import tf

SHAPE_ARRAY_TOPIC_NAME="/shape_out"

class Visualization:
    def __init__(self):
        rospy.Subscriber(SHAPE_ARRAY_TOPIC_NAME, MarkerArray, self.marker_cb)
        self.pub = rospy.Publisher('automatica_marker', MarkerArray)

    def marker_cb(self, msg):
        print "got new shape"
        marker_array = MarkerArray()
        
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        pose.position.z = 0.5

#        marker_array.markers = self.axes_marker_array(pose).markers
        marker_array.markers += self.complete_box_marker(pose, 1).markers
        pose2 = copy.deepcopy(pose)
        pose2.position.x += 0.2
        marker_array.markers += self.complete_box_marker(pose2, 2).markers

        self.pub.publish(marker_array)


    def complete_box_marker(self, pose, id):
        marker_array = self.axes_marker_array(pose, id)
        marker_array.markers.append(self.box_marker(pose, id))
        return marker_array
        
    def box_marker(self, pose, id):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/base_link"
        marker.ns = "boxes"
        marker.id = id
        marker.type = 1
        marker.pose = copy.deepcopy(pose)
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0) #sec 
        return marker

    def axes_marker_array(self, pose, id):
        axes_marker_array = MarkerArray()
        x = Marker()
        x.header.stamp = rospy.Time.now()
        x.header.frame_id = "/base_link"
        x.ns = "axes"
        x.id = id+1000
        x.type = 0
        x.pose = copy.deepcopy(pose)
        x.pose.position.z += 0.025
        x.scale.x = 0.1
        x.scale.y = 0.01
        x.scale.z = 0.01
        x.color.r = 1.0
        x.color.g = 0.0
        x.color.b = 0.0
        x.color.a = 1.0
        x.lifetime = rospy.Duration(1.0) #sec 
        
        y = Marker()
        y.header.stamp = rospy.Time.now()
        y.header.frame_id = "/base_link"
        y.ns = "axes"
        y.id = id + 1001
        y.type = 0
        y.pose = copy.deepcopy(pose)
        y.pose.position.z += 0.025
#TODO rotate 90deg around z-axis
#        euler_tuple = tf.transformations.euler_from_quaternion([y.pose.orientation.x, y.pose.orientation.y, y.pose.orientation.z, y.pose.orientation.w])
#        euler_list = [euler_tuple[0], euler_tuple[1], euler_tuple[2]]
#        print euler_list
#        euler_list[1] += 1.5708
#        print tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
#        y.pose.orientation = tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
        y.scale.x = 0.1
        y.scale.y = 0.01
        y.scale.z = 0.01
        y.color.r = 0.0
        y.color.g = 1.0
        y.color.b = 0.0
        y.color.a = 1.0
        y.lifetime = rospy.Duration(1.0) #sec 
        
        z = Marker()
        z.header.stamp = rospy.Time.now()
        z.header.frame_id = "/base_link"
        z.ns = "axes"
        z.id = id + 1002
        z.type = 0
        z.pose = copy.deepcopy(pose)
        z.pose.position.z += 0.025
#TODO rotate 90deg around x-axis
        z.scale.x = 0.1
        z.scale.y = 0.01
        z.scale.z = 0.01
        z.color.r = 0.0
        z.color.g = 0.0
        z.color.b = 1.0
        z.color.a = 1.0
        z.lifetime = rospy.Duration(1.0) #sec 
        
        axes_marker_array.markers.append(x)
        axes_marker_array.markers.append(y)
        axes_marker_array.markers.append(z)
        return axes_marker_array


if __name__ == '__main__':
    rospy.init_node('lacquey_gripper')
    print "starting visualization"
    server = Visualization()
    print "visualization started"
    rospy.spin()
    
    

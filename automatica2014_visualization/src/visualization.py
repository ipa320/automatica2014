#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_visualization')

import sys

import rospy
import copy
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from cob_3d_mapping_msgs.msg import *
import tf

SHAPE_ARRAY_TOPIC_NAME="/shapes_array_obj_out"
LIFETIME=0.1
FRAMEID="cam3d_env_link"

class Visualization:
    def __init__(self):
        rospy.Subscriber(SHAPE_ARRAY_TOPIC_NAME, ShapeArray, self.marker_cb)
        self.pub = rospy.Publisher('automatica_marker', MarkerArray)
        self.listener = tf.TransformListener()

    def marker_cb(self, msg):
        print "got new shape: ", len(msg.shapes)
        
        marker_array = MarkerArray()
        
        id_count = 0
        for shape in msg.shapes:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = FRAMEID
            ps.pose = shape.pose
            ps_base_link = self.listener.transformPose("base_link", ps)
            
            # HACK: fix position and orientations (hardcoded)
            ps_base_link.pose.position.z = 0.05
            #TODO set roll and pitch to 0 (fixed), only allow variation in yaw
            
            marker_array.markers += self.complete_box_marker(ps_base_link, id_count).markers
            id_count += 1

        self.pub.publish(marker_array)


    def complete_box_marker(self, ps, id):
        marker_array = self.axes_marker_array(ps, id)
        marker_array.markers.append(self.box_marker(ps, id))
        return marker_array
        
    def box_marker(self, ps, id):
        marker = Marker()
        marker.header.stamp = ps.header.stamp
        marker.header.frame_id = ps.header.frame_id
        marker.ns = "boxes"
        marker.id = id
        marker.type = 1
        marker.pose = copy.deepcopy(ps.pose)
        marker.scale.x = 0.045
        marker.scale.y = 0.10
        marker.scale.z = 0.045
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(LIFETIME) #sec 
        return marker

    def axes_marker_array(self, ps, id):       
        axes_marker_array = MarkerArray()
        x = Marker()
        x.header.stamp = ps.header.stamp
        x.header.frame_id = ps.header.frame_id
        x.ns = "axes"
        x.id = id+1000
        x.type = 0
        x.pose = copy.deepcopy(ps.pose)
        x.pose.position.z += 0.025
        x.scale.x = 0.1
        x.scale.y = 0.01
        x.scale.z = 0.01
        x.color.r = 1.0
        x.color.g = 0.0
        x.color.b = 0.0
        x.color.a = 1.0
        x.lifetime = rospy.Duration(LIFETIME) #sec 
        
        y = Marker()
        y.header.stamp = ps.header.stamp
        y.header.frame_id = ps.header.frame_id
        y.ns = "axes"
        y.id = id + 1001
        y.type = 0
        y.pose = copy.deepcopy(ps.pose)
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
        y.lifetime = rospy.Duration(LIFETIME) #sec 
        
        z = Marker()
        z.header.stamp = ps.header.stamp
        z.header.frame_id = ps.header.frame_id
        z.ns = "axes"
        z.id = id + 1002
        z.type = 0
        z.pose = copy.deepcopy(ps.pose)
        z.pose.position.z += 0.025
#TODO rotate 90deg around x-axis
        z.scale.x = 0.1
        z.scale.y = 0.01
        z.scale.z = 0.01
        z.color.r = 0.0
        z.color.g = 0.0
        z.color.b = 1.0
        z.color.a = 1.0
        z.lifetime = rospy.Duration(LIFETIME) #sec 
        
        axes_marker_array.markers.append(x)
#        axes_marker_array.markers.append(y)
#        axes_marker_array.markers.append(z)
        return axes_marker_array


if __name__ == '__main__':
    rospy.init_node('lacquey_gripper')
    print "starting visualization"
    server = Visualization()
    print "visualization started"
    rospy.spin()
    
    

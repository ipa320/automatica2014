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
LIFETIME=1.0
FRAMEID="cam3d_env_link"

class Visualization:
    def __init__(self):
        rospy.Subscriber(SHAPE_ARRAY_TOPIC_NAME, ShapeArray, self.marker_cb)
        self.pub_marker = rospy.Publisher('automatica_marker', MarkerArray)
        self.pub_poses = rospy.Publisher('automatica_poses', PoseArray)
        self.listener = tf.TransformListener()

    def marker_cb(self, msg):
        
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "base_link"
        
        id_count = 0
        for shape in msg.shapes:
            ps = PoseStamped()
            ps.header.stamp = pose_array.header.stamp
            ps.header.frame_id = FRAMEID
            ps.pose = shape.pose
            ps_base_link = self.listener.transformPose("base_link", ps)
            
            # HACK: fix position and orientations (hardcoded)
            ps_base_link.pose.position.z = 0.10

            euler_tuple = tf.transformations.euler_from_quaternion([ps_base_link.pose.orientation.x, ps_base_link.pose.orientation.y, ps_base_link.pose.orientation.z, ps_base_link.pose.orientation.w])
            euler_list = [euler_tuple[0], euler_tuple[1], euler_tuple[2]]
            euler_list[0] = 0
            euler_list[1] = 0
            quat_x, quat_y, quat_z, quat_w = tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
            ps_base_link.pose.orientation.x = quat_x
            ps_base_link.pose.orientation.y = quat_y
            ps_base_link.pose.orientation.z = quat_z
            ps_base_link.pose.orientation.w = quat_w
            # end HACK
            
            
            
            #TODO set roll and pitch to 0 (fixed), only allow variation in yaw
            
            marker_array.markers += self.complete_box_marker(ps_base_link, id_count).markers
            pose_array.poses.append(ps_base_link.pose)
            id_count += 1

        self.pub_marker.publish(marker_array)
        self.pub_poses.publish(pose_array)


    def complete_box_marker(self, ps, id):
        marker_array = self.axes_marker_array(ps, id)
        marker_array.markers.append(self.box_marker(ps, id))
        return marker_array
        
    def box_marker(self, ps, id):
        marker = Marker()
        marker.header.stamp = ps.header.stamp
        marker.header.frame_id = ps.header.frame_id
        marker.ns = "boxes"
        marker.id = id*1000
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
        x.id = id*1000+1
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
        y.id = id*1000+2
        y.type = 0
        y.pose = copy.deepcopy(ps.pose)
        y.pose.position.z += 0.025
        # rotate 90deg around z-axis
        euler_tuple = tf.transformations.euler_from_quaternion([y.pose.orientation.x, y.pose.orientation.y, y.pose.orientation.z, y.pose.orientation.w])
        euler_list = [euler_tuple[0], euler_tuple[1], euler_tuple[2]]
        euler_list[2] += 1.5708
        quat_x, quat_y, quat_z, quat_w = tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
        y.pose.orientation.x = quat_x
        y.pose.orientation.y = quat_y
        y.pose.orientation.z = quat_z
        y.pose.orientation.w = quat_w
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
        z.id = id*1000+3
        z.type = 0
        z.pose = copy.deepcopy(ps.pose)
        z.pose.position.z += 0.025
        # rotate -90deg around y-axis
        euler_tuple = tf.transformations.euler_from_quaternion([z.pose.orientation.x, z.pose.orientation.y, z.pose.orientation.z, z.pose.orientation.w])
        euler_list = [euler_tuple[0], euler_tuple[1], euler_tuple[2]]
        euler_list[1] += -1.5708
        quat_x, quat_y, quat_z, quat_w = tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2])
        z.pose.orientation.x = quat_x
        z.pose.orientation.y = quat_y
        z.pose.orientation.z = quat_z
        z.pose.orientation.w = quat_w
        z.scale.x = 0.1
        z.scale.y = 0.01
        z.scale.z = 0.01
        z.color.r = 0.0
        z.color.g = 0.0
        z.color.b = 1.0
        z.color.a = 1.0
        z.lifetime = rospy.Duration(LIFETIME) #sec 

        
        
        
        
        
        
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
    
    

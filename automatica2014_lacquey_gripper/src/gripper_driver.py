#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_lacquey_gripper')

import sys

import rospy
import actionlib
from ur_msgs.srv import *
from control_msgs.msg import *
from sensor_msgs.msg import *

FJT_ACTION_NAME="lacquey_controller/follow_joint_trajectory"
SETIO_SERVICE_NAME="set_io"


class GripperAction:
    def __init__(self):
        self._as = actionlib.SimpleActionServer(FJT_ACTION_NAME, FollowJointTrajectoryAction, self.execute_cb, False)
        self._as.start()
        self._set_io = rospy.ServiceProxy(SETIO_SERVICE_NAME, SetIOState)
        self._pub_joint_states = rospy.Publisher('joint_states', JointState)
        self._joint_states = JointState()
        self._joint_states.name = ["gripper_joint"]
        self._joint_states.position = [0.0]
        self._joint_states.velocity = [0.0]
        self._joint_states.effort = [0.0]
        
        # activate_gripper
        req = SetIOStateRequest()
        req.state.pin = 3 #TODO
        req.state.state = 0 #TODO
        # TODO call service


    def execute_cb(self, goal):
        print "action called"
        
        position = goal.trajectory.points[0].positions[0]
        if position <= 0.5: # close
            print "closing gripper"
            self._joint_states.position = [0.0] #TODO
            req = SetIOStateRequest()
            req.state.pin = 3 #TODO
            req.state.state = 0 #TODO
        else: # open
            print "open gripper"
            self._joint_states.position = [1.0] #TODO
            req = SetIOStateRequest()
            req.state.pin = 3 #TODO
            req.state.state = 1 #TODO
        
        # call IO Service from UR5 robot
        try:
            rospy.wait_for_service(SETIO_SERVICE_NAME,3)
        except rospy.ROSException:
            print "Service %s not available" %SETIO_SERVICE_NAME
            self._as.set_aborted()
            return
        
        try:
            
            self._set_io(req)
        except rospy.ServiceException:
            print "Service call failed, aborting action"
            self._as.set_aborted()
            return

        # all went well, return succeeded
        self._as.set_succeeded()
        print "action finished"
        
    def publish_joint_states(self):
        self._joint_states.header.stamp = rospy.Time.now()
        self._pub_joint_states.publish(self._joint_states)


if __name__ == '__main__':
    rospy.init_node('lacquey_gripper')
    print "starting gripper"
    server = GripperAction()
    print "gripper started"
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        server.publish_joint_states()
        r.sleep()
    
    
    

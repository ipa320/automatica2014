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


# open: pin 8 state 1, pin 9 state 0
# close: pin 8 state 0, pin 9 state 1


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
        
        # activate_gripper (open)
        self.move_gripper("open")

    # "open" or "close"
    # else error
    def move_gripper(position):
            req8 = SetIOStateRequest()
            req8.state.pin = 8
            req9 = SetIOStateRequest()
            req9.state.pin = 9
        if position = "open": #close
            req8.state.state = 1
            req9.state.state = 0
        elif position == "close"
            req8.state.state = 0
            req9.state.state = 1
        else:
            print "wrong gipper command"
            return false
        
        # call IO Service from UR5 robot
        try:
            rospy.wait_for_service(SETIO_SERVICE_NAME,3)
        except rospy.ROSException:
            print "Service %s not available" %SETIO_SERVICE_NAME
            return False
        
        try:
            self._set_io(req8)
        except rospy.ServiceException:
            print "Service call failed, aborting action"
            return False

        try:
            self._set_io(req9)
        except rospy.ServiceException:
            print "Service call failed, aborting action"
            return False
        
        return True

    def execute_cb(self, goal):
        print "action called"
        
        position = goal.trajectory.points[0].positions[0]
        if position <= 0.5: # close
            print "closing gripper"
            self._joint_states.position = [0.0] #TODO
            success = self.move_gripper("close")
        else: # open
            print "open gripper"
            self._joint_states.position = [1.0] #TODO
            success = self.move_gripper("open")
        
        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()
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
    
    
    

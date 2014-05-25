#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_lacquey_gripper')

import sys

import rospy
import actionlib
from ur_msgs.srv import *
from control_msgs.msg import *
from sensor_msgs.msg import *

FJT_ACTION_NAME="gripper_controller/follow_joint_trajectory"
SETIO_SERVICE_NAME="arm_controller/set_io_state"
POSITION_OPEN = 1.0
POSITION_CLOSE = 0.01

# open: pin 8 state 1, pin 9 state 0
# close: pin 8 state 0, pin 9 state 1


class GripperAction:
    def __init__(self):
        self._as = actionlib.SimpleActionServer(FJT_ACTION_NAME, FollowJointTrajectoryAction, self.execute_cb, False)
        self._as.start()
        self._set_io = rospy.ServiceProxy(SETIO_SERVICE_NAME, SetIOState)
        self._pub_joint_states = rospy.Publisher('joint_states', JointState)
        self._pub_controller_state = rospy.Publisher('gripper_controller/state', JointTrajectoryControllerState)
        # joint state
        self._joint_states = JointState()
        self._joint_states.name = ["finger_left_joint"]
        self._joint_states.position = [POSITION_OPEN]
        self._joint_states.velocity = [0.0]
        self._joint_states.effort = [0.0]
        # contoller state
        self._controller_state = JointTrajectoryControllerState()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = self._joint_states.name
        self._controller_state.actual.positions = self._joint_states.position
        self._controller_state.actual.velocities = self._joint_states.velocity        
        
        # activate_gripper (open)
        # self.move_gripper("open")

    # "open" or "close"
    # else error
    def move_gripper(self, position):
        req8 = SetIOStateRequest()
        req8.state.pin = 8
        req9 = SetIOStateRequest()
        req9.state.pin = 9
        if position == "open": #open
            req8.state.state = 1
            req9.state.state = 0
        elif position == "close":
            req8.state.state = 0
            req9.state.state = 1
        else:
            print "wrong gipper command"
            return False
        
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
        rospy.sleep(0.5)
        return True

    def execute_cb(self, goal):
        print "action called"
        
        position = goal.trajectory.points[-1].positions[0]
        if position <= 0.02: # close
            print "closing gripper"
            self._joint_states.position = [POSITION_CLOSE]
            self._controller_state.actual.positions = [POSITION_CLOSE] #TODO
            success = self.move_gripper("close")
        else: # open
            print "open gripper"
            self._joint_states.position = [POSITION_OPEN] #TODO
            self._controller_state.actual.positions = [POSITION_OPEN] #TODO
            success = self.move_gripper("open")
        
        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()
        print "action finished"
        
    def publish_joint_states(self):
        self._joint_states.header.stamp = rospy.Time.now()
        self._pub_joint_states.publish(self._joint_states)
    
    def publish_controller_state(self):
        self._controller_state.header.stamp = rospy.Time.now()
        self._pub_controller_state.publish(self._controller_state)


if __name__ == '__main__':
    rospy.init_node('lacquey_gripper')
    print "starting gripper"
    server = GripperAction()
    print "gripper started"
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        server.publish_joint_states()
        server.publish_controller_state()
        r.sleep()
    
    
    

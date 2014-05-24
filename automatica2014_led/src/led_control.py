#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_led')

import sys

import rospy
import actionlib
from ur_msgs.srv import *
from cob_msgs.srv import *

LED_ACTION_NAME = "led_control/set_mode"
SETIO_SERVICE_NAME="arm_controller/set_io_state"
mode_on = 1
mode_off = 0

# on: pin 2 state 1
# off: pin 2 state 0


class LedAction:
    def __init__(self):
        self._as = actionlib.SimpleActionServer(LED_ACTION_NAME, setString, self.execute_cb, False)
        self._as.start()
        self._set_io = rospy.ServiceProxy(SETIO_SERVICE_NAME, SetIOState)   
        
        # activate_Led (on)
        self.set_led("on")

    # "on" or "off"
    # else error
    def set_led(self, mode):
        req2 = SetIOStateRequest()
        req2.state.pin = 2
        if mode == "on": #off
            req2.state.state = mode_on
        elif mode == "off":
            req2.state.state = mode_off
        else:
            print "wrong led command"
            return False
        
        # call IO Service from UR5 robot
        try:
            rospy.wait_for_service(SETIO_SERVICE_NAME,3)
        except rospy.ROSException:
            print "Service %s not available" %SETIO_SERVICE_NAME
            return False
        
        try:
            self._set_io(req2)
        except rospy.ServiceException:
            print "Service call failed, aborting action"
            return False
             
        return True

    def execute_cb(self, goal):
        print "action called"
        
        mode = goal.data
        if mode == mode_off: # off
            print "turning off led"
           
            success = self.set_led("off")
        else: # on
            print "turning the led on"
            success = self.set_led("on")
        
        if success:
            self._as.set_succeeded()
        else:
            self._as.set_aborted()
        print "action finished"
        
   # def publish_joint_states(self):
   #     self._joint_states.header.stamp = rospy.Time.now()
   #     self._pub_joint_states.publish(self._joint_states)
    
   # def publish_controller_state(self):
   #     self._controller_state.header.stamp = rospy.Time.now()
   #     self._pub_controller_state.publish(self._controller_state)


if __name__ == '__main__':
    rospy.init_node('automatica_led')
    print "starting Led"
    server = LedAction()
    print "Led control started"
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    #    server.publish_joint_states()
    #    server.publish_controller_state()
        r.sleep()
    
    
    

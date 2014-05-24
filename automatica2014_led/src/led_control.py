#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_led')

import sys

import rospy
import actionlib
from ur_msgs.srv import *
from cob_srvs.srv import *

LED_ACTION_NAME = "led_control/set_mode"
SETIO_SERVICE_NAME="arm_controller/set_io_state"
mode_on = 1
mode_off = 0

# on: pin 2 state 1
# off: pin 2 state 0


class LedAction:
    def __init__(self):
        self._as = rospy.Service(LED_ACTION_NAME, SetString, self.execute_cb)
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
        print mode
        if mode == "off": # off
            print "turning off led"
           
            success = self.set_led("off")
        else: # on
            print "turning the led on"
            success = self.set_led("on")
        
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
    rospy.wait_for_service('push_buttons_node/button_state', 2.0)
    
    prev_led_mode = "Button OFF"
    while not rospy.is_shutdown():
        try:
            get_button_state = rospy.ServiceProxy('push_buttons_node/button_state', SetString)
            resp1 = get_button_state("sia10f")
            led_mode = resp1.errorMessage.data
            if(led_mode != prev_led_mode):
                if(led_mode == "Button OFF"):
                    server.set_led("off")
                elif(led_mode=="Button ON"):
                    server.set_led("on")
                    
            prev_led_mode = led_mode
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    #    server.publish_joint_states()
    #    server.publish_controller_state()
        r.sleep()
    
    
    

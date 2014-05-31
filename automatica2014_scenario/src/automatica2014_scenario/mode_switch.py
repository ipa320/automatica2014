#! /usr/bin/env python2

import rospy
from cob_phidgets.msg import DigitalSensor
from ur_msgs.srv import SetIOState, SetIOStateRequest
import threading

class ModeSwitch:
    PINS = {'sia10f': 0, 'ur5': 1}
    BLINK = [0,1]
    URIS = { '324299/0' : 'sia10f', '324299/1' : 'ur5' }
    def __init__(self, cb=None):
        self.cb = cb
        self.lock = threading.Lock()
        
        self.current_mode = ''
        self.requested_mode = ''

        self.srv_set_io = rospy.ServiceProxy('/ur5/arm_controller/set_io_state', SetIOState, persistent = True)
        
        self.io_state = SetIOStateRequest()
        self.io_state.state.pin = self.BLINK[0]
        self.timer = rospy.Timer(rospy.Duration(1), self.blink)
        
        rospy.Subscriber("/digital_sensors", DigitalSensor, self.sensors_cb)

    def _call_set_io_state(self, pin, state):
        self.io_state.state.pin = pin
        self.io_state.state.state = 1.0 if state else 0.0
        try:
            self.srv_set_io(self.io_state)
            #print self.io_state
        except rospy.ServiceException as ex:
            #print ex
            self.srv_set_io = rospy.ServiceProxy('/ur5/arm_controller/set_io_state', SetIOState, persistent = True)
        
        
    def _set_mode_led(self, mode, state):
        if mode:
            self._call_set_io_state(self.PINS[mode], state)
        else:
            self._call_set_io_state(self.io_state.state.pin, state)
            
            
    def blink(self, event):
        with self.lock:
            try:
                pin = self.PINS[self.requested_mode]
                self._call_set_io_state(pin, 0.0 if self.io_state.state.state  else 1.0)
            except KeyError:
                next_pin = self.BLINK[0] # for start over
                try:
                    next_pin  = self.BLINK[self.BLINK.index(self.io_state.state.pin)+1]
                except IndexError:
                    pass
                self._call_set_io_state(self.io_state.state.pin, 0)
                self._call_set_io_state(next_pin, 1)
                
                
                
    def sensors_cb(self, msg):
        new_mode = ''
        old_mode = ''
        with self.lock:
            if self.current_mode == self.requested_mode:
                if msg.state[0] == 0:
                    self.requested_mode = self.URIS[msg.uri[0]]
            if self.current_mode != self.requested_mode:
                self.timer.shutdown()
                if not self.current_mode:
                    self._call_set_io_state(self.io_state.state.pin, False)
                self._set_mode_led(self.requested_mode, True)
                self.timer = rospy.Timer(rospy.Duration(1), self.blink)
                new_mode = self.requested_mode
                old_mode = self.current_mode
        if new_mode and self.cb:
            self.cb(old_mode, new_mode)
            
    def set_mode(self, mode):
        with self.lock:
            self.timer.shutdown()
            self._set_mode_led(self.current_mode, False)
            self._set_mode_led(mode, True)
            self.current_mode = mode
            self.requested_mode = mode
            if not mode:
                self.timer = rospy.Timer(rospy.Duration(1), self.blink)
if __name__ == "__main__":
    rospy.init_node("test")
    test = None
    def enable_mode(event):
        test.set_mode(test.requested_mode)
    def print_modes(before, after):
        print before, after
        rospy.Timer(rospy.Duration(5),enable_mode, True)
    test = ModeSwitch(print_modes)
    rospy.spin()


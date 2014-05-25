#!/usr/bin/env python
import roslib; roslib.load_manifest('automatica2014_pg70_fake_joint')

import sys

import rospy

from cob_srvs.srv import *

class AutoInit_wait:
    def __init__(self):
        rospy.wait_for_service("/sia10f/gripper_controller/init", 10.0)
        

if __name__ == '__main__':
    rospy.init_node('pg70_auto_init')
    print "auto initializing gripper"
    server = AutoInit_wait()
    auto_init = rospy.ServiceProxy('/sia10f/gripper_controller/init', Trigger)
    auto_init()
    print "auto initialized gripper"
    
    


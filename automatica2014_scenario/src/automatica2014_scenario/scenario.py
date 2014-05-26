#! /usr/bin/env python2

import rospy
from automatica2014_scenario.pick_place import MoveitInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from cob_phidgets.msg import *
from copy import deepcopy

_current_poses = None
_active_arm = None
_switch_active = False
def poses_cb(msg):
    global _current_poses
    _current_poses = msg.poses

def sensors_cb(msg):
    global _switch_active, _active_arm
    if not _switch_active and msg.state[0] == 0:
        if msg.uri[0] == "324299/0" and not _active_arm == "sda10f": # left=sda10f
            _active_arm = "sda10f"
        elif msg.uri[0] == "324299/1" and not _active_arm == "ur5": # right=ur5
            _active_arm = "ur5"
        else:
            #print "invalid request"
            return
        print "switching to arm: ", _active_arm
        _switch_active = True
    elif msg.state[0] == 1:
        return
    else:
        print "I'm currently switching the application to another arm. Please press later again. \n Active arm: ", _active_arm


if __name__ == "__main__":
    rospy.init_node("scenario")
    MI_sia10f = MoveitInterface("sia10f")
    MI_ur5 = MoveitInterface("ur5")
    print "moveit interfaces initialized"
    
    rospy.Subscriber("/automatica_poses", PoseArray, poses_cb)
    rospy.Subscriber("/digital_sensors", DigitalSensor, sensors_cb)
    
    """
    for i in range(10):
        if _current_poses:
            print "pick", i
            print _current_poses
            
            ret = True#test.pick_one_of(deepcopy(_current_poses))
            print ret
            _current_poses = None
            if ret:
                break
        rospy.sleep(0.5)
    
    for i in range(10):
        print "place", i
        ret = False#test.place(0.94,0.9,0)
        if ret == 1:
            break
        rospy.sleep(0.5)
    """
    while not rospy.is_shutdown():
        if _current_poses and _active_arm and _switch_active:
            _current_poses = None
            _switch_active = False
            print "pick and place with ", _active_arm
            rospy.sleep(3)
        rospy.sleep(0.5)

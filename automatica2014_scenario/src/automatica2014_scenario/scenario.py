#! /usr/bin/env python2

import rospy
from automatica2014_scenario.pick_place import MoveitInterface
from automatica2014_scenario.mode_switch import ModeSwitch


from geometry_msgs.msg import PoseStamped, PoseArray
from copy import deepcopy
import threading

import smach
import smach_ros

import sys

class ChooseRobotState(smach.State):
    
    def __init__(self, robot_interfaces):
        smach.State.__init__(self,
            outcomes = ['done','error'],
            input_keys = [],
            output_keys = ['active_robot'])
        self.mode_switch = ModeSwitch(self.switch)
        self.condition = threading.Condition()
        self.active_robot = ''
        self.robot_interfaces = robot_interfaces
    def switch(self, old_robot, new_robot):
        if old_robot:
            self.robot_interfaces[old_robot].cancel()
        with self.condition:
            self.active_robot = new_robot
            self.condition.notify()
        
    def execute(self, userdata):
        self.active_robot = ''
        self.mode_switch.set_mode(self.active_robot)
        while not self.active_robot:
            with self.condition:
                self.condition.wait()
        if True: #try:
            print self.active_robot
            userdata.active_robot = self.active_robot
            self.mode_switch.set_mode(self.active_robot)
            return 'done'
        #except:
        else:
            print sys.exc_info()
        return 'error'

class MoveHomeState(smach.State):
    def __init__(self, robot_interfaces, home_targets):
        smach.State.__init__(self,
                outcomes = ['done','error'],
                input_keys = ['active_robot','blocked_poses','pickable_poses'])
        self.robot_interfaces = robot_interfaces
        self.home_targets = home_targets
    def execute(self, userdata):
        if False: #userdata.robot_interface.move(*self.home_targets[userdata.active_robot], others = userdata.blocked_poses + userdata.pickable_poses ) == 1:
           return 'done'
        else:
           return 'error'
        
class SelectObjectsState(smach.State):
    CANDIATES_BACK = [ (0,0), (0,0), (0,0), (0,0),]
    CANDIATES_FRONT = [ (1,0), (1,0), (1,0), (1,0),]
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['done','not_found','error'],
            input_keys = [],
            output_keys = ['blocked_poses', 'pickable_poses','place_candidates'])
        self.lock = threading.Lock()
        self.poses = []
        self.sub_poses = rospy.Subscriber("/automatica_poses", PoseArray, self.poses_cb)
        self.pick = 'front'
       
    def poses_cb(self, msg):
        with self.lock:
            self.poses = msg.poses
            
    def execute(self, userdata):
        back = []
        front = []
        with self.lock:
            for p in self.poses:
                if p.x > 0.5:
                    front.append(p)
                else:
                    back.append(p)
        if len(front) == 0 and len(back) == 0:
            userdata.blocked_poses = []
            userdata.pickable_poses = []
            userdata.place_candidates = []
            return 'not_found'
        elif self.pick == 'front' and len(front) == 0:
            self.pick = 'back'
        elif self.pick == 'back' and len(back) == 0:
            self.pick = 'front'
            
        if self.pick == 'front':
            userdata.blocked_poses = back
            userdata.pickable_poses = front
            userdata.place_candidates = self.CANDIATES_BACK
        elif self.pick == 'front':
            userdata.blocked_poses = front
            userdata.pickable_poses = back
            userdata.place_candidates = self.CANDIATES_FRONT
        return 'done'

class PickState(smach.State):
    def __init__(self, robot_interfaces):
        smach.State.__init__(self,
                outcomes = ['done','not_reached', 'error'],
                input_keys = ['active_robot', 'blocked_poses','pickable_poses'])
        self.robot_interfaces = robot_interfaces
    def execute(self, userdata):
        res = self.robot_interfaces[userdata.active_robot].pick_one_of(userdata.pickable_poses, userdata.blocked_poses)
        if res == 1:
           return 'done'
        elif res == 2: # TODO
            return 'not_reached'
        else:
           return 'error'
        
class PlaceState(smach.State):
    def __init__(self, robot_interfaces):
        smach.State.__init__(self,
                outcomes = ['done','not_reached', 'error'],
                input_keys = ['active_robot', 'blocked_poses','pickable_poses'])
        self.robot_interfaces = robot_interfaces
    def execute(self, userdata):
        res = self.robot_interfaces[userdata.active_robot].place_somewhere(userdata.place_candidates, userdata.pickable_poses + userdata.blocked_poses)
        if res == 1:
           return 'done'
        elif res == 2: # TODO
            return 'not_reached'
        else:
           return 'error'
        
if __name__ == "__main__":
    rospy.init_node("scenario")
    
    robot_interfaces  = {
        'sia10f' : MoveitInterface("sia10f", [0.034,0.34],[0.001,0.001]),
        'ur5':  MoveitInterface("ur5", [1.0,1.0],[0.0,0.0]),
    }
    home_targets = {
            'sia10f': (['sia10f_'+l for l in ['joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t']],[0,0,0,0,0,0,0]),
            'ur5': (['ur5_'+l for l in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']],[0,0,0,0,0,0,0]),
    }
    

    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CHOOSE_ROBOT', ChooseRobotState(robot_interfaces), 
    transitions={'done':'SELECT_OBJECTS','error':'exit'})
        smach.StateMachine.add('SELECT_OBJECTS', SelectObjectsState(), 
    transitions={'done':'PICK_OBJECT','not_found':'MOVE_HOME','error':'exit'})
        smach.StateMachine.add('PICK_OBJECT', PickState(robot_interfaces), 
    transitions={'done':'PLACE_OBJECT','not_reached':'MOVE_HOME','error':'exit'})
        smach.StateMachine.add('PLACE_OBJECT', PlaceState(robot_interfaces), 
    transitions={'done':'SELECT_OBJECTS','not_reached':'MOVE_HOME','error':'exit'})
        smach.StateMachine.add('MOVE_HOME', MoveHomeState(robot_interfaces, home_targets), 
        transitions={'done':'CHOOSE_ROBOT','error':'exit'})

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('Automatica2014', sm, '/')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()    
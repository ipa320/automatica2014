#! /usr/bin/env python2

import rospy
import actionlib
import tf.transformations
from moveit_msgs.msg import *
from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive, Plane
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from actionlib_msgs.msg import GoalStatus 
def make_plane(name, pose, normal = (0, 0, 1), offset = 0):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    p = Plane()
    p.coef = list(normal)
    p.coef.append(offset)
    co.planes = [p]
    co.plane_poses = [pose.pose]
    return co

def make_joint_trajectory(names, points):
    jt = JointTrajectory()
    jt.joint_names = names
    pt = JointTrajectoryPoint()
    pt.positions = points
    pt.effort = [0]*len(points)
    pt.velocities = [0]*len(points)
    pt.accelerations = [0]*len(points)
    jt.points = [pt]
    return jt
    
def make_top_grasp(pose):
    # a list of possible grasps to be used. At least one grasp must be filled in
    #manipulation_msgs/Grasp[] possible_grasps
    grasp = Grasp()
    print grasp
    grasp.id = "top"

    # open
    grasp.pre_grasp_posture = make_joint_trajectory(['finger_left_joint','finger_right_joint'],[1,1])

    # close
    grasp.grasp_posture = make_joint_trajectory(['finger_left_joint','finger_right_joint'],[0.01,0.01])

    grasp.grasp_pose = pose

    grasp.pre_grasp_approach.direction.header.stamp = rospy.Time.now()
    grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
    grasp.pre_grasp_approach.direction.vector.z = -1.0
    grasp.pre_grasp_approach.desired_distance = 0.10
    grasp.pre_grasp_approach.min_distance = 0.05
    
    grasp.post_grasp_retreat.direction.header.stamp = rospy.Time.now()
    grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.desired_distance = 0.10
    grasp.post_grasp_retreat.min_distance = 0.05
    
    grasp.post_place_retreat = deepcopy(grasp.pre_grasp_approach)
    return grasp
    
def make_pickup_goal(poses):
    goal = PickupGoal()
    goal.target_name = "object"
    goal.group_name = "arm"
    goal.end_effector = "gripper"

    #float32 max_contact_force

    #string[] allowed_touch_objects
    goal.possible_grasps = [make_top_grasp(pose) for pose in poses] 
    
    goal.support_surface_name = "table"
    goal.allow_gripper_support_collision = False
    # The maximum amount of time the motion planner is allowed to plan for
    goal.allowed_planning_time = 1.0
    goal.attached_object_touch_links = ['gripper_link','finger_left_link','finger_right_link']
    return goal

def make_box(name, pose, size = (0, 0, 1)):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)
    co.primitives = [box]
    co.primitive_poses = [pose.pose]
    return co
        
class MoveitInterface:
    TABLE_HEIGHT = 0.01
    OBJECT_HEIGHT = 0.04
    OBJECT_LENGTH = 0.1
    def __init__(self, ns):
        self.ns = ns
        self.pub_co = rospy.Publisher(ns+'/collision_object', CollisionObject)
        self.srv_ps  = rospy.ServiceProxy(ns + '/get_planning_scene', GetPlanningScene)  
        self.action_pickup = actionlib.SimpleActionClient(ns+'/pickup', PickupAction)
        rospy.sleep(2.0)
        
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "base_link"
        p.pose.position.x = 0.6
        p.pose.position.y = 1.00
        p.pose.position.z = self.TABLE_HEIGHT-0.005 
        p.pose.orientation.w = 1.0
        self.pub_co.publish(make_box("table",p,(1.2,0.7,0.01)))
        
    def is_grasped(self):
        res = self.srv_ps(GetPlanningSceneRequest(PlanningSceneComponents(4)))
        return len(res.scene.robot_state.attached_collision_objects) > 0
        
    def pick_one_of(self, poses):
        for pose in poses:
            r,p,y = tf.transformations.euler_from_quaternion(pose.orientation)
            res = self.pick(pose.position.x,pose.position.y,y)
            if res in [MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.NO_IK_SOLUTION]:
                continue
            else:
                break
        return res == MoveItErrorCodes.SUCCESS
                     
    def pick(self, x,y, alpha):
        if self.is_grasped():
           print "grasped"
           return MoveItErrorCodes.SUCCESS
        else:
            print "not grasped"
            p = PoseStamped()
            p.header.frame_id = "base_link"
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = self.TABLE_HEIGHT + self.OBJECT_HEIGHT/2.0
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha)
            self.pub_co.publish(make_box("object", p, (self.OBJECT_HEIGHT,self.OBJECT_LENGTH,self.OBJECT_HEIGHT)))
            
            p2 = deepcopy(p)
            p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha + math.pi)
            
            goal = make_pickup_goal([p,p2])
            
            ok = self.action_pickup.send_goal_and_wait(goal)
            
            if ok == GoalStatus.PREEMPTED:
                return MoveItErrorCodes.PREEMPTED
            if ok != GoalStatus.SUCCEEDED:
                return MoveItErrorCodes.FAILURE
            res = self.action_pickup.get_result()
            return res.error_code.val
if __name__ == "__main__":
    rospy.init_node("test")
    test = MoveitInterface("")
    test.pick(0.5,1.0,0)
    rospy.spin()

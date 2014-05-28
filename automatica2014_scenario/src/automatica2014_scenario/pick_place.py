#! /usr/bin/env python2

import rospy
import actionlib
import tf.transformations
from moveit_msgs.msg import *
from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive, Plane
from geometry_msgs.msg import PoseStamped, PoseArray
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
    grasp.id = "top"

    # open
    #grasp.pre_grasp_posture = make_joint_trajectory(['finger_left_joint','finger_right_joint'],[1.0,1.0])
    grasp.pre_grasp_posture = make_joint_trajectory(['finger_left_joint','finger_right_joint'],[0.03,0.03])

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
    goal.allowed_planning_time = 5.0
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    
    
    goal.attached_object_touch_links = ['gripper_link','finger_left_link','finger_right_link']
    return goal

def make_place_goal(poses):
    goal = PlaceGoal()
    goal.group_name = "gripper"
    goal.attached_object_name = "object"
    
    approach = GripperTranslation()
    approach.direction.header.stamp = rospy.Time.now()
    approach.direction.header.frame_id = "base_link"
    approach.direction.vector.z = -1.0
    approach.desired_distance = 0.10
    approach.min_distance = 0.05

    retreat = GripperTranslation()
    retreat.direction.header.stamp = rospy.Time.now()
    retreat.direction.header.frame_id = "base_link"
    retreat.direction.vector.z = 1.0
    retreat.desired_distance = 0.10
    retreat.min_distance = 0.05

    goal.place_locations = [ PlaceLocation(place_pose=p,pre_place_approach=approach, post_place_retreat=retreat)  for p in poses ]
    
    goal.support_surface_name = "table"
    goal.allowed_planning_time = 5.0

    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal

def make_move_goal(joints,values):
    goal = MoveGroupGoal()
    goal.group_name = arm
    goal.allowed_planning_time = 5.0
    
    c = Constraints()
    for (n,v) in zip(joints,values):
        jc = JointConstraint()
        jc.name = j
        jc.position = v
        jc.tolerance_above = jc.tolerance_below = 0.005
    c.joint_constraints.append(jc)
    goal.goal_constraints = [c]

    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal

def add_box(co, pose, size = (0, 0, 1), offset=(0,0,0)):
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)
    co.primitives.append(box)
    p = deepcopy(pose)
    p.position.x += offset[0]
    p.position.y += offset[1]
    p.position.z += offset[2]
    co.primitive_poses.append(p)
    return co

def make_box(name, pose, size = (0, 0, 1), offset=(0,0,0)):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    add_box(co, pose.pose, size,offset)
    return co
        
        
class MoveitInterface:
    TABLE_HEIGHT = 0.01
    OBJECT_HEIGHT = 0.05
    OBJECT_LENGTH = 0.15
    TABLE1 = (0.37, 1.045, 0)
    TABLE2 = (0.945, 1.045, 0)
    TABLE_SIZE = (0.4,0.6,0.02,0.01, 0.07) # x y border, floor, wall
    
    def init_table(self):
        x, y, border, floor, wall = self.TABLE_SIZE
        
        p1 = PoseStamped()
        p1.header.frame_id = "base_link"
        p1.header.stamp = rospy.Time.now()
        p1.pose.position.x, p1.pose.position.y, alpha1 = self.TABLE1
        p1.pose.position.z = self.TABLE_HEIGHT
        p1.pose.orientation.x,p1.pose.orientation.y,p1.pose.orientation.z,p1.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha1)

        p2 = PoseStamped()
        p2.header.frame_id = "base_link"
        p2.header.stamp = rospy.Time.now()
        p2.pose.position.x, p2.pose.position.y, alpha2 = self.TABLE2
        p2.pose.position.z = self.TABLE_HEIGHT
        p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha2)
        
        table = make_box("table",p1,(x,y,floor),(0,0,-floor/2.0))
        add_box(table,p2.pose,(x,y,floor),(0,0,floor/2.0))
        self.pub_co.publish(table)
        
        wall1 = make_box("wall1",p1,(border,y,wall),(-(x/2.0-border/2.0),0,wall/2.0)) # back
        add_box(wall1,p1.pose, (border,y,wall),(+(x/2.0-border/2.0),0,wall/2.0)) # front
        add_box(wall1,p1.pose, (x,border,wall),(0,+(y/2.0-border/2.0),wall/2.0)) # right
        add_box(wall1,p1.pose, (x,border,wall),(0,-(y/2.0-border/2.0),wall/2.0)) # left
        self.pub_co.publish(wall1)

        wall2 = make_box("wall2",p2,(border,y,wall),(-(x/2.0-border/2.0),0,wall/2.0)) # back
        add_box(wall2,p2.pose, (border,y,wall),(+(x/2.0-border/2.0),0,wall/2.0)) # front
        add_box(wall2,p2.pose, (x,border,wall),(0,+(y/2.0-border/2.0),wall/2.0)) # right
        add_box(wall2,p2.pose, (x,border,wall),(0,-(y/2.0-border/2.0),wall/2.0)) # left
        self.pub_co.publish(wall2)
            
    def __init__(self, ns):
        self.ns = ns
        self.pub_co = rospy.Publisher(ns+'/collision_object', CollisionObject)
        self.srv_ps  = rospy.ServiceProxy(ns + '/get_planning_scene', GetPlanningScene)  
        self.action_pickup = actionlib.SimpleActionClient(ns+'/pickup', PickupAction)
        self.action_place = actionlib.SimpleActionClient(ns+'/place', PlaceAction)
        self.action_move = actionlib.SimpleActionClient(ns+'/move_group', MoveGroupAction)
        rospy.sleep(2.0)
        self.transit = False
        
        self.init_table()
        
    def is_grasped(self):
        res = self.srv_ps(GetPlanningSceneRequest(PlanningSceneComponents(4)))
        return len(res.scene.robot_state.attached_collision_objects) > 0
        
    def pick_one_of(self, poses, others):
        shuffle(poses)
        while not poses.empty():
            pose = poses.pop()
            r,p,y = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y , pose.orientation.z, pose.orientation.w])
            res = self.pick(pose.position.x,pose.position.y,y, others + poses)
            others.append(pose)
            if res in [MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.NO_IK_SOLUTION]:
                continue
            else:
                break
        return res == MoveItErrorCodes.SUCCESS
        
    def make_collision_objects(self, poses):
        objs = []
        i = 0
        for p in poses:
            _,_,angle = tf.transformations.euler_from_quaternion([p.pose.orientation.x, p.pose.orientation.y , p.pose.orientation.z, p.pose.orientation.w])
            objs.append(make_box("object_"+str(i), self.make_object_pose(p.pose.position.x,p.pose.position.y,angle), (self.OBJECT_HEIGHT,self.OBJECT_LENGTH,self.OBJECT_HEIGHT)))
            i += 1
        return objs
        
    def make_object_pose(self, x, y, alpha, offset = 0, header = None):
        p = PoseStamped()
        if not header:
            p.header.frame_id = "base_link"
            p.header.stamp = rospy.Time.now()
        else:
            p.header = header
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = self.TABLE_HEIGHT + self.OBJECT_HEIGHT/2.0 + offset
        p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha)
        return p
        
    def move(names,values, others = []):
        if not self.transit:
            return
        self.transit = True
        self.action_pickup.cancel()
        self.action_pickup.cancel()

        goal = make_move_goal(names, values)
        goal.planning_options.planning_scene_diff.world.collision_objects = self.make_collision_objects(others)
        
        ok = self.action_pickup.send_goal_and_wait(goal)
        if ok != GoalStatus.SUCCEEDED:
            return false
        res = self.action_move.get_result()
        self.transit = False
        return res.error_code.val == MoveItErrorCodes.SUCCESS

    def pick(self, x,y, alpha, others= [], force = False):
        if self.is_grasped():
           print "already grasped"
           return MoveItErrorCodes.SUCCESS
           if not force:
               return MoveItErrorCodes.SUCCESS
        p = self.make_object_pose(x,y,alpha)
        self.pub_co.publish(make_box("object", p, (self.OBJECT_HEIGHT*2,self.OBJECT_LENGTH,self.OBJECT_HEIGHT)))
        
        p2 = deepcopy(p)
        p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha + math.pi)
        
        goal = make_pickup_goal([p,p2])
        goal.planning_options.planning_scene_diff.world.collision_objects = self.make_collision_objects(others)
        
        ok = self.action_pickup.send_goal_and_wait(goal)
        
        if ok == GoalStatus.PREEMPTED:
            return MoveItErrorCodes.PREEMPTED
        if ok != GoalStatus.SUCCEEDED:
            return MoveItErrorCodes.FAILURE
        res = self.action_pickup.get_result()
        return res.error_code.val
        
    def place_somewhere(self, poses, others=[]):
        
        pl = []
        for pose in poses:
            _,_,alpha = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y , pose.orientation.z, pose.orientation.w])
            pl.append(pose)
            for i in range(6):
                p2 = deepcopy(pose)
                p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha + math.pi/3)
                pl.append(p2)
        shuffle(pl)

        goal = make_place_goal([p,p2])
        goal.planning_options.planning_scene_diff.world.collision_objects = self.make_collision_objects(others)

        ok = self.action_place.send_goal_and_wait(goal)
        
        if ok != GoalStatus.SUCCEEDED:
            return False
        res = self.action_place.get_result()
        return res.error_code.val == MoveItErrorCodes.SUCCESS
        
        return res == MoveItErrorCodes.SUCCESS
    def place(self, x,y, alpha, others=[], force = False):
        if not self.is_grasped():
           print "not grasped"
           if not force:
               return MoveItErrorCodes.SUCCESS
        p = self.make_object_pose(x,y,alpha,0.005)
        p2 = deepcopy(p)
        p2.pose.orientation.x,p2.pose.orientation.y,p2.pose.orientation.z,p2.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,alpha + math.pi)
        
        goal = make_place_goal([p,p2])
        goal.planning_options.planning_scene_diff.world.collision_objects = self.make_collision_objects(others)
        
        ok = self.action_place.send_goal_and_wait(goal)
        
        if ok == GoalStatus.PREEMPTED:
            return MoveItErrorCodes.PREEMPTED
        if ok != GoalStatus.SUCCEEDED:
            return MoveItErrorCodes.FAILURE
        res = self.action_place.get_result()
        return res.error_code.val

if __name__ == "__main__":
    rospy.init_node("test")
    test = MoveitInterface("")
    
    others = [ test.make_object_pose(0.94,0.645,0) ]
    while not rospy.is_shutdown():
        print "pick", test.pick(0.35,1.0,0)
        rospy.sleep(0.5)
        print "place", test.place(0.94,1.045,0,others)
        rospy.sleep(0.5)
    rospy.spin()

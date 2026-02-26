#!/usr/bin/env python3
import rospy
import py_trees
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

class Blackboard:
    target_pose = None
    current_pose = None
    current_state = None

bb = Blackboard()

class CheckCollision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckCollision, self).__init__(name)
        # Placeholder: In a real app, subscribe to /scan or /camera/depth
    
    def update(self):
        # Return FAILURE if no collision, allowing the Selector to proceed to Mission
        # Return SUCCESS/RUNNING if collision detected to preempt Mission
        return py_trees.common.Status.FAILURE

class WaitForGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(WaitForGoal, self).__init__(name)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        bb.target_pose = msg
        rospy.loginfo("New goal received: x=%.2f, y=%.2f", msg.pose.position.x, msg.pose.position.y)

    def update(self):
        if bb.target_pose is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class ArmAndOffboard(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ArmAndOffboard, self).__init__(name)
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    def update(self):
        if bb.current_state is None:
            return py_trees.common.Status.FAILURE

        # Keep publishing setpoint to allow offboard switch
        if bb.target_pose:
            target = bb.target_pose
            target.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(target)

        if not bb.current_state.armed:
            try:
                self.arming_client(True)
            except rospy.ServiceException:
                pass
        
        if bb.current_state.mode != "GUIDED" and bb.current_state.mode != "OFFBOARD":
            try:
                # Try GUIDED for ArduPilot, OFFBOARD for PX4
                self.set_mode_client(base_mode=0, custom_mode="GUIDED")
            except rospy.ServiceException:
                pass

        if bb.current_state.armed and (bb.current_state.mode == "GUIDED" or bb.current_state.mode == "OFFBOARD"):
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

class FlyToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FlyToGoal, self).__init__(name)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    def update(self):
        if not bb.target_pose or not bb.current_pose:
            return py_trees.common.Status.FAILURE

        # Publish Setpoint
        target = bb.target_pose
        target.header.stamp = rospy.Time.now()
        self.local_pos_pub.publish(target)

        # Check Distance
        dx = target.pose.position.x - bb.current_pose.pose.position.x
        dy = target.pose.position.y - bb.current_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 1.0:
            rospy.loginfo("Target Reached!")
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

class ReturnToLaunch(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ReturnToLaunch, self).__init__(name)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def update(self):
        if bb.current_state.mode == "RTL":
            # Reset target so we can wait for a new one after landing
            bb.target_pose = None 
            return py_trees.common.Status.SUCCESS
        
        try:
            self.set_mode_client(base_mode=0, custom_mode="RTL")
            rospy.loginfo("Switching to RTL")
        except rospy.ServiceException:
            pass
            
        return py_trees.common.Status.RUNNING

def pose_cb(msg):
    bb.current_pose = msg

def state_cb(msg):
    bb.current_state = msg

def create_tree():
    root = py_trees.composites.Selector("DroneLogic")
    
    # 1. Collision Avoidance (Highest Priority)
    root.add_child(CheckCollision("CollisionAvoidance"))
    
    # 2. Mission Sequence
    mission = py_trees.composites.Sequence("Mission")
    mission.add_child(WaitForGoal("WaitForGoal"))
    mission.add_child(ArmAndOffboard("ArmAndTakeoff"))
    mission.add_child(FlyToGoal("FlyToGoal"))
    mission.add_child(ReturnToLaunch("ReturnToLaunch"))
    
    root.add_child(mission)
    return root

if __name__ == '__main__':
    rospy.init_node('fly_to_point_node')
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/mavros/state", State, state_cb)
    
    tree = py_trees.trees.BehaviourTree(create_tree())
    tree.setup(timeout=15)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()
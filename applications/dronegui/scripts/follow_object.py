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
    mission_start_time = None

bb = Blackboard()

class CheckCollision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckCollision, self).__init__(name)
        # Placeholder: Subscribe to /scan or /camera/depth in a real scenario
    
    def update(self):
        # Return FAILURE if no collision, allowing the Selector to proceed to Mission
        # Return SUCCESS/RUNNING if collision detected to preempt Mission
        return py_trees.common.Status.FAILURE

class WaitForObject(py_trees.behaviour.Behaviour):
    """Waits for the first object pose to be received."""
    def __init__(self, name):
        super(WaitForObject, self).__init__(name)
        self.object_sub = rospy.Subscriber("/external_object/pose", PoseStamped, self.object_callback)

    def object_callback(self, msg):
        bb.target_pose = msg
        # Initialize start time on first receipt if not set
        if bb.mission_start_time is None:
            bb.mission_start_time = rospy.Time.now()
            rospy.loginfo("Object detected. Mission timer started.")

    def update(self):
        if bb.target_pose is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class CheckTimeLimit(py_trees.behaviour.Behaviour):
    """Returns SUCCESS if time is within limit, FAILURE if time expired."""
    def __init__(self, name, duration_sec=300):
        super(CheckTimeLimit, self).__init__(name)
        self.duration = rospy.Duration(duration_sec)

    def update(self):
        if bb.mission_start_time is None:
            return py_trees.common.Status.FAILURE
        
        elapsed = rospy.Time.now() - bb.mission_start_time
        if elapsed < self.duration:
            return py_trees.common.Status.SUCCESS
        else:
            rospy.loginfo_throttle(5, "Mission time expired. Switching to RTL branch.")
            return py_trees.common.Status.FAILURE

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
            # Override Z to fly OVER the object (e.g., 10m altitude)
            target.pose.position.z = 10.0
            self.local_pos_pub.publish(target)

        if not bb.current_state.armed:
            try:
                self.arming_client(True)
            except rospy.ServiceException:
                pass
        
        if bb.current_state.mode != "GUIDED" and bb.current_state.mode != "OFFBOARD":
            try:
                self.set_mode_client(base_mode=0, custom_mode="GUIDED")
            except rospy.ServiceException:
                pass

        if bb.current_state.armed and (bb.current_state.mode == "GUIDED" or bb.current_state.mode == "OFFBOARD"):
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

class FollowObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FollowObject, self).__init__(name)
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    def update(self):
        if not bb.target_pose:
            return py_trees.common.Status.FAILURE

        # Fly directly over the object
        target = bb.target_pose
        target.header.stamp = rospy.Time.now()
        target.pose.position.z = 10.0 # Maintain 10m altitude
        
        self.local_pos_pub.publish(target)
        return py_trees.common.Status.RUNNING

class ReturnToLaunch(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ReturnToLaunch, self).__init__(name)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def update(self):
        if bb.current_state.mode == "RTL":
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
    
    # 2a. Wait for object detection
    mission.add_child(WaitForObject("WaitForObject"))
    
    # 2b. Follow Logic (Selector: Try to follow, if time fails, do RTL)
    follow_or_rtl = py_trees.composites.Selector("FollowOrRTL")
    
    # 2b-1. Active Following (Sequence)
    active_follow = py_trees.composites.Sequence("ActiveFollow")
    active_follow.add_child(CheckTimeLimit("Check5MinLimit", duration_sec=300))
    active_follow.add_child(ArmAndOffboard("ArmAndTakeoff"))
    active_follow.add_child(FollowObject("FollowObject"))
    
    follow_or_rtl.add_child(active_follow)
    
    # 2b-2. Return to Launch (Fallback when time expires)
    follow_or_rtl.add_child(ReturnToLaunch("ReturnToLaunch"))
    
    mission.add_child(follow_or_rtl)
    
    root.add_child(mission)
    return root

if __name__ == '__main__':
    rospy.init_node('follow_object_node')
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/mavros/state", State, state_cb)
    
    tree = py_trees.trees.BehaviourTree(create_tree())
    tree.setup(timeout=15)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tree.tick()
        rate.sleep()
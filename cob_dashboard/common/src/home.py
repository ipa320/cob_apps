#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_dashboard')
import rospy
import actionlib

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

def home():
    rospy.init_node('move_arm')
    client = actionlib.SimpleActionClient('arm_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    # Fill in the goal
    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    
    # First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names=["joint_arm1","joint_arm2","joint_arm3","joint_arm4","joint_arm5","joint_arm6","joint_arm7"]

    # here you specify a list of trajectory points
    point=JointTrajectoryPoint()
    point.positions=[0,0,0,0,0,0,0]
    point.velocities=[0,0,0,0,0,0,0]
    point.time_from_start=rospy.Duration(3)
    goal.trajectory.points.append(point)

    # print goal for debugging reasons
    print "start moving on trajectory"
    #print goal

    client.send_goal(goal)
#    client.wait_for_result()
#    print "movement finished"

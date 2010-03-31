#!/usr/bin/env python
import roslib; roslib.load_manifest('cob3_dashboard')
import rospy
import actionlib

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

def torso_front():
    rospy.init_node('move_torso')
    client = actionlib.SimpleActionClient('torso_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    # Fill in the goal
    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    
    # First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names=["joint_platform_neckZ","joint_neckZ_neck","joint_neck_headZ","joint_headZ_head"]

    # here you specify a list of trajectory points
    point=JointTrajectoryPoint()
    point.positions=[0,0.2,0,0.2]
    point.velocities=[0,0,0,0]
    point.time_from_start=rospy.Duration(3)
    goal.trajectory.points.append(point)

    # print goal for debugging reasons
    print "start moving on trajectory"
    #print goal

    client.send_goal(goal)
#    client.wait_for_result()
#    print "movement finished"

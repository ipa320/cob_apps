#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
import actionlib
import time
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from cob_actions.msg import *
from cob_msgs.msg import *
from trajectory_msgs.msg import *


rospy.init_node('dual_arm_script')
rospy.sleep(1.0)

cob_lbr_client = actionlib.SimpleActionClient('/lbr_controller/joint_trajectory_action', JointTrajectoryAction)
lwr_client = actionlib.SimpleActionClient('/lwr/lbr_controller/joint_trajectory_action', JointTrajectoryAction)

cob_sdh_client = actionlib.SimpleActionClient('/sdh_controller/joint_trajectory_action', JointTrajectoryAction)
lwr_sdh_client = actionlib.SimpleActionClient('/lwr/sdh_controller/joint_trajectory_action', JointTrajectoryAction)

rospy.sleep(2.0)

lbr_joint_names = ["lbr_1_joint","lbr_2_joint","lbr_3_joint","lbr_4_joint","lbr_5_joint","lbr_6_joint","lbr_7_joint"]
sdh_joint_names = ["sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_11_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_21_joint", "sdh_finger_22_joint", "sdh_finger_23_joint"]

home = JointTrajectory()
home.joint_names = lbr_joint_names
point=JointTrajectoryPoint()
point.positions=[0,0,0,0,0,0,0]
point.time_from_start=rospy.Duration(3)
home.points.append(point)

grasp = JointTrajectory()
grasp.joint_names = lbr_joint_names
point=JointTrajectoryPoint()
point.positions=[0.5, 0.8, 0.8, -0.8, 0.8, 0.8, -0.3]
point.time_from_start=rospy.Duration(3)
grasp.points.append(point)

cylOpen = JointTrajectory()
cylOpen.joint_names = sdh_joint_names
point=JointTrajectoryPoint()
point.positions=[-0.7854,1.0472,0.0,-0.7854,1.0472,0.0,-0.7854,1.0472]
point.time_from_start=rospy.Duration(2)
cylOpen.points.append(point)

cylClose = JointTrajectory()
cylClose.joint_names = sdh_joint_names
point=JointTrajectoryPoint()
point.positions=[0.0,1.0472,0.0,0.0,1.0472,0.0,0.0,1.0472]
point.time_from_start=rospy.Duration(2)
cylClose.points.append(point)

goal = JointTrajectoryGoal()
sdhgoal = JointTrajectoryGoal()

while not rospy.is_shutdown():
    goal.trajectory = home
    goal.trajectory.header.stamp = rospy.Time.now()
    cob_lbr_client.send_goal(goal)
    lwr_client.send_goal(goal)
    cob_lbr_client.wait_for_result()
    lwr_client.wait_for_result()

    sdhgoal.trajectory = cylOpen
    sdhgoal.trajectory.header.stamp = rospy.Time.now()
    cob_sdh_client.send_goal(sdhgoal)
    lwr_sdh_client.send_goal(sdhgoal)
    lwr_sdh_client.wait_for_result()
    cob_sdh_client.wait_for_result()
    

    goal.trajectory = grasp
    goal.trajectory.header.stamp = rospy.Time.now()
    cob_lbr_client.send_goal(goal)
    lwr_client.send_goal(goal)
    cob_lbr_client.wait_for_result()
    lwr_client.wait_for_result()

    sdhgoal.trajectory = cylClose
    sdhgoal.trajectory.header.stamp = rospy.Time.now()
    cob_sdh_client.send_goal(sdhgoal)
    lwr_sdh_client.send_goal(sdhgoal)
    lwr_sdh_client.wait_for_result()
    cob_sdh_client.wait_for_result()

#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_mmcontroller')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
def dual_arm_test():
    pub = rospy.Publisher('/lwr/lbr_controller/command', JointTrajectory)
    cob_pub = rospy.Publisher('/cob3_1/lbr_controller/command', JointTrajectory)
    rospy.init_node('dual_arm_test')
    rospy.sleep(1.0)
    while not rospy.is_shutdown():
        traj = JointTrajectory()
        traj.joint_names = ['lbr_1_joint', 'lbr_2_joint', 'lbr_3_joint', 'lbr_4_joint', 'lbr_5_joint', 'lbr_6_joint', 'lbr_7_joint']
        ptn = JointTrajectoryPoint()
        ptn.positions = [0.5, 0.8, 0.8, -0.8, 0.8, 0.8, -0.3]
        ptn.velocities = [0.4, 0.4, 0.8, 0.1, 0.8, 0.8, 0.1]
        ptn.time_from_start = rospy.Duration(2.0)
        traj.header.stamp = rospy.Time.now()
        traj.points.append(ptn)
        pub.publish(traj)
        cob_pub.publish(traj)
        rospy.sleep(3.0)
        ptn.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ptn.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        ptn.time_from_start = rospy.Duration(2.0)
        traj.points = []
        traj.header.stamp = rospy.Time.now()
        traj.points.append(ptn)
        pub.publish(traj)
        cob_pub.publish(traj)
        rospy.sleep(3.0)

if __name__ == '__main__':
    try:
        dual_arm_test()
    except rospy.ROSInterruptException: pass

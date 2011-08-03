#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_component_test')
import sys
import time
import unittest
import rospy
import rostest
import actionlib
import time

from simple_script_server import *
from geometry_msgs.msg import *
from actionlib_msgs.msg import *
from move_base_msgs import *
from tf.transformations import *

NAME = 'cobbase_unit'
class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node(NAME)
        self.sss=simple_script_server()
    #def setUp(self):
    #    self.errors = []
    def test_unit(self):
        # fetch parameters
        try:
            # time of test
            test_duration = float(rospy.get_param('~test_duration'))
            # x
            x = float(rospy.get_param('~x_value'))      
            # y
            y = float(rospy.get_param('~y_value'))      
            # theta
            theta = float(rospy.get_param('~theta_value'))      
        except KeyError, e:
            self.fail('cobunit not initialized properly')
        print """
              X: %s
              Y: %s
              Theta: %s
              Test duration:%s"""%(x, y, theta, test_duration)
        self._test_base(x, y, theta, test_duration)
        
    def _test_base(self, x, y, theta, test_duration): 
        self.assert_(test_duration > 0.0, "bad parameter (test_duration)")
        rospy.sleep(5.0)
        self.sss.init("arm")
        self.sss.move("arm","folded")
        #print "hello world"
        client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        client_goal = MoveBaseGoal()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        client_goal.target_pose = pose
        client.wait_for_server()
        client.send_goal(client_goal)
        while client.get_state() != 3:
        	rospy.sleep(1.0)
        	if client.get_state() == 0:
        		client.send_goal(client_goal)
         
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"


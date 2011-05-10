#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_component_test')
#roslib.load_manifest('cob_script_server')

import sys
import time
import unittest
import rospy
import rostest
import actionlib
import tf
#import cob_gazebo

from simple_script_server import *
from gazebo.msg import *
from geometry_msgs.msg import *

from math import *

class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node('grasp_test')
        self.message_received = False
        self.sss=simple_script_server()
        
    def setUp(self):
        self.errors = []
        
    def test_grasp(self):
        # get parameters
        # more can be included later
        try:
            # test duration
            if not rospy.has_param('~test_duration'):
                self.fail('Parameter test_duration does not exist on ROS Parameter Server')
            test_duration = rospy.get_param('~test_duration')
            
            # model name of object to grasp
            if not rospy.has_param('~grasp_object'):
                self.fail('Parameter grasp_object does not exist on ROS Parameter Server')
            grasp_object = rospy.get_param('~grasp_object')
        
        except KeyError, e:
            self.fail('Parameters not set properly')
        
        print """
            Test duration: %s
            Object to grasp: %s"""%(test_duration, grasp_object)
            
        # init subscribers
        sub_model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb_model_states)
        sub_link_states = rospy.Subscriber("/gazebo/link_states", LinkStates, self.cb_link_states)
                
        # transformation handle        
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        
        # init components and check initialization
        components_to_init = ['arm', 'tray', 'sdh', 'torso', 'base'] 
        for component in components_to_init:
            init_component = self.sss.init(component)
            if init_component.get_error_code() != 0:
                error_msg = 'Could not initialize ' + component
                self.fail(error_msg)
         
        # check if grasp_object was spawned correctly
        if grasp_object not in self.model_states.name:
            self.fail(grasp_object + " not spawned correctly")
            
        # get index of grasp_object in topic
        obj_index = self.model_states.name.index(grasp_object)
        
        # get index of arm_7_link in topic
        self.arm_7_link_index = self.link_states.name.index("arm_7_link")
             
        # move robot in position to grasp object
        # base kitchen
        handle_base = self.sss.move('base', 'kitchen', False)
        # tray down
        self.sss.move('tray', 'down', False)
        # arm pregrasp
        self.sss.move('arm', 'pregrasp', False)
        # sdh cylopen
        self.sss.move('sdh', 'cylopen', False)
        handle_base.wait()
        
        # transform object coordinates 
        grasp_obj = self.trans_into_arm_7_link(self.model_states.pose[obj_index].position)
        print >> sys.stderr, grasp_obj.point           # to be deleted
        
        # move in front of object
        pregrasp_distance = 0.03
        grasp_offset = 0.17 # offset between arm_7_link and sdh_grasp_link
#        self.sss.move_cart_rel("arm", [[0.03, grasp_obj.point.y - 0.02, (grasp_obj.point.z - grasp_offset - 0.4)], [0.0, 0.0, 0.0]])
        # move to object
#        self.sss.move_cart_rel("arm", [[0.0, 0.0, 0.22], [0.0, 0.0, 0.0]])
        self.sss.move_cart_rel("arm", [[0.0, 0.0, 0.2], [0.0, 0.0, 0.0]])
        
        # grasp object
        self.sss.move("sdh", grasp_object)
        # lift object
        self.sss.move_cart_rel("arm", [[0.2, -0.1, -0.2], [0.0, 0.0, 0.0]])
               
        # check object position + status message
        self.check_pos(self.link_states.pose[self.arm_7_link_index].position, self.model_states.pose[obj_index].position, 0.5, "sdh in kitchen")

        # move arm over tray
        handle_arm = self.sss.move("arm", "grasp-to-tray", False)        
        # tray up
        self.sss.move("tray", "up")
        handle_arm.wait()

        # check object position + status message
        self.check_pos(self.link_states.pose[self.arm_7_link_index].position, self.model_states.pose[obj_index].position, 0.5, "sdh over tray")
        
		# put object onto tray
        self.sss.move_cart_rel("arm", [[-0.05, 0.0, 0.0], [0, 0, 0]])
        self.sss.move("sdh", "cylopen")
        
        # base kitchen
        self.sss.move('base', [0, -0.5, 0])
        
        # check object position + status message
        des_pos_world = Point()
        des_pos_world.x = 0.3
        des_pos_world.y = -0.4
        des_pos_world.z = 0.885
        self.check_pos(des_pos_world, self.model_states.pose[obj_index].position, 0.5, "tray")
        
        # grasp objekt on tray
        self.sss.move("sdh", grasp_object)
        
        # put object to final position
        self.sss.move("arm", "overtray")
        self.sss.move_cart_rel("arm", [[0.0, 0.0, -0.2], [0, 0, 0]])
        self.sss.move("arm", [[1.4272207239645427, -0.86918345596744029, -2.6785592907972724, -0.83566556448023821, 0.93072293274776374, 1.2925104647818602, -2.3042322384962883]])
        des_pos_world.x = 0.0
        des_pos_world.y = -1.2
        des_pos_world.z = 0.56  
        des_pos_arm_7_link = self.trans_into_arm_7_link(des_pos_world)
        # calculate the distance between object bottom and arm_7_link 
        x_dist_obj_arm_7_link = self.trans_into_arm_7_link(self.model_states.pose[obj_index].position).point.x
        self.sss.move_cart_rel("arm", [[(des_pos_arm_7_link.point.x - x_dist_obj_arm_7_link + 0.01), des_pos_arm_7_link.point.y, des_pos_arm_7_link.point.z], [0.0, 0.0, 0.0]])
        self.sss.move("sdh", "cylopen")
        
        # check object position + status message
        des_pos_world.z = 0.55
        self.check_pos(des_pos_world, self.model_states.pose[obj_index].position, 0.2, "table")
        
        self.sss.move_cart_rel("arm", [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.sss.move("arm", "folded")
     
        
        
    # callback functions
    def cb_model_states(self, msg):
        self.model_states = msg
        
    def cb_link_states(self, msg):
        self.link_states = msg
        
    def check_pos(self, des_pos, act_pos, tolerance, pos_name):
        # function to check if object is at desired position
        distance = self.calc_dist(des_pos, act_pos)
        if distance >= tolerance:
            error_msg = "Object not at desired position '" + pos_name + "'"
            self.fail(error_msg)
        else:
            print >> sys.stderr, "Object in/on '", pos_name, "'"
        print >> sys.stderr, "Distance to '", pos_name, "': ", distance
        
    def calc_dist(self, des_pos, act_pos):
        # function to calculate distance between actual and desired object position
        distance = sqrt((des_pos.x - act_pos.x)**2 + (des_pos.y - act_pos.y)**2 + (des_pos.z - act_pos.z)**2)
        return distance
        
        
    def trans_into_arm_7_link(self, coord_world):
        # function to transform given coordinates into arm_7_link coordinates
        coord_arm_7_link = PointStamped()
        coord_arm_7_link.header.stamp = rospy.Time.now()
        coord_arm_7_link.header.frame_id = "/map"
        coord_arm_7_link.point = coord_world
        self.sss.sleep(2) # wait for transform to be calculated
        
        if not self.sss.parse:
            coord_arm_7_link = self.listener.transformPoint('/arm_7_link', coord_arm_7_link)
            # transform grasp point to sdh center
#            coord_arm_7_link.point.z = coord_arm_7_link.point.z - 0.2
        return coord_arm_7_link
            
        
if __name__ == '__main__':
    try:
        rostest.run('rostest', 'grasp_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

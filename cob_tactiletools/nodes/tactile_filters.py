#!/usr/bin/python
import roslib; roslib.load_manifest('cob_tactiletools')
import rospy
from cob_sdh.msg import TactileMatrix, TactileSensor
from cob_srvs.srv import *
from std_msgs.msg import Bool, Float32MultiArray



class TactileFilters():
    def __init__(self):
        mean_value_publisher = 0
        grabbed_publisher = 0
        cylindric_grabbed_publisher = 0
        touched_treshold = 0
        self.is_grasped = False
        self.is_cylindric_grasped = False
        self.one_pad_contact = False
    
    def getMean(self, tarray):
        sum = 0
        for i in tarray:
            sum += i
        return sum/len(tarray)
            

    def roscb(self, data):
        matrices = data.tactile_matrix
        meanvalues = Float32MultiArray()
        touched = 0
        touched_finger = 0
        touched_thumb = 0
        meanvalues.data = []
        for m in matrices:
            meanvalues.data.insert(m.matrix_id, self.getMean(m.tactile_array))
            if(meanvalues.data[m.matrix_id] > self.touched_treshold):
                touched += 1
                if((m.matrix_id == 2) or (m.matrix_id == 3)):
                    touched_thumb += 1
                else:
                    touched_finger += 1
        self.mean_value_publisher.publish(meanvalues)
        if(touched >= 2):
            self.is_grasped = True
            self.grabbed_publisher.publish(Bool(True))
        else:
            self.is_grasped = False
            self.grabbed_publisher.publish(Bool(False))
        if((touched_thumb >= 1) and (touched_finger >= 1)):
            self.is_cylindric_grasped = True
            self.cylindric_grabbed_publisher.publish(Bool(True))
        else:
            self.is_cylindric_grasped = False
            self.cylindric_grabbed_publisher.publish(Bool(False))
        if(touched >= 1):
        	self.one_pad_contact = True
        else:
        	self.one_pad_contact = False
        
    def handle_is_grasped(self, req):
        res = TriggerResponse()
        if self.is_grasped == True:
            res.success.data = True
            res.error_message.data = "grasped object"
        else:
            res.success.data = False
            res.error_message.data= "object not grasped"
        # print "status: is_grasped = ",self.is_grasped,", success = ",res.success
        return res

    def handle_is_cylindric_grasped(self, req):
        res = TriggerResponse()
        if self.is_cylindric_grasped == True:
            res.success.data = True
            res.error_message.data = "grasped object"
        else:
            res.success.data = False
            res.error_message.data= "object not grasped"
        # print "status: is_cylindric_grasped = ",self.is_cylindric_grasped,", success = ",res.success
        return res
        
    def handle_one_pad_contact(self, req):
        res = TriggerResponse()
        res.success.data = self.one_pad_contact
        return res

if (__name__ == "__main__"):
    TF = TactileFilters()
    rospy.init_node('TactileSensorView', anonymous=True)
    rospy.Subscriber("/sdh_controller/tactile_data", TactileSensor, TF.roscb)
    TF.mean_value_publisher = rospy.Publisher("/sdh_controller/mean_values", Float32MultiArray)
    TF.grabbed_publisher = rospy.Publisher("/sdh_controller/grabbed", Bool)
    service_is_grasped = rospy.Service('/sdh_controller/is_grasped', Trigger, TF.handle_is_grasped)
    TF.cylindric_grabbed_publisher = rospy.Publisher("/sdh_controller/cylindric_grabbed", Bool)
    service_is_cylindric_grasped = rospy.Service('/sdh_controller/is_cylindric_grasped', Trigger, TF.handle_is_cylindric_grasped)
    service_one_pad_contact = rospy.Service('/sdh_controller/one_pad_contact', Trigger, TF.handle_one_pad_contact)
    treshold = rospy.get_param('TouchedTreshold', '10')
    TF.touched_treshold = treshold
    print "Setting touched treshold to ", treshold
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
    

#!/usr/bin/python
import roslib; roslib.load_manifest('cob_tactiletools')
import rospy
from cob_msgs.msg import TactileMatrix, TactileSensor
from cob_srvs.srv import *
from std_msgs.msg import Bool, Float32MultiArray



class TactileFilters():
    def __init__(self):
        mean_value_publisher = 0
        grabbed_publisher = 0
        touched_treshold = 0
        self.is_grasped = False
    
    def getMean(self, tarray):
        sum = 0
        for i in tarray:
            sum += i
        return sum/len(tarray)
            

    def roscb(self, data):
        matrices = data.tactile_matrix
        meanvalues = Float32MultiArray()
        touched = 0
        meanvalues.data = []
        for m in matrices:
            meanvalues.data.insert(m.matrix_id, self.getMean(m.tactile_array))
            if(meanvalues.data[m.matrix_id] > self.touched_treshold):
                touched += 1
        self.mean_value_publisher.publish(meanvalues)
        if(touched >= 2):
            self.is_grasped = True
            self.grabbed_publisher.publish(Bool(True))
        else:
            self.is_grasped = False
            self.grabbed_publisher.publish(Bool(False))
        
    def handle_is_grasped(self, req):
        res = TriggerResponse()
        if self.is_grasped == True:
            res.success = 0
            res.errorMessage.data = "grasped object"
        else:
            res.success = 1
            res.errorMessage.data = "object not grasped"
        print "status: is_grasped = ",self.is_grasped,", success = ",res.success
        return res


if (__name__ == "__main__"):
    TF = TactileFilters()
    rospy.init_node('TactileSensorView', anonymous=True)
    rospy.Subscriber("/sdh/tactile_data", TactileSensor, TF.roscb)
    TF.mean_value_publisher = rospy.Publisher("/tactile_tools/mean_values", Float32MultiArray)
    TF.grabbed_publisher = rospy.Publisher("/tactile_tools/grabbed", Bool)
    service_is_grasped = rospy.Service('tactile_tools/is_grasped', Trigger, TF.handle_is_grasped)
    treshold = rospy.get_param('TouchedTreshold', '10')
    TF.touched_treshold = treshold
    print "Setting touched treshold to ", treshold
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
    

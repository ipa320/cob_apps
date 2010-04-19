import roslib; roslib.load_manifest('cob_tactiletools')
import rospy
from cob_msgs.msg import TactileMatrix, TactileSensor
from std_msgs.msg import Bool, Float32MultiArray



class TactileFilters():
    def __init__(self):
        mean_value_publisher = 0
        grabbed_publisher = 0
    
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
            if(meanvalues.data[m.matrix_id] > 0):
                touched += 1
        self.mean_value_publisher.publish(meanvalues)
        if(touched >= 2):
            self.grabbed_publisher.publish(Bool(True))
        else:
            self.grabbed_publisher.publish(Bool(False))
        






if (__name__ == "__main__"):
    TF = TactileFilters()
    rospy.init_node('TactileSensorView', anonymous=True)
    rospy.Subscriber("/sdh/tactile_data", TactileSensor, TF.roscb)
    TF.mean_value_publisher = rospy.Publisher("/tactile_tools/mean_values", Float32MultiArray)
    TF.grabbed_publisher = rospy.Publisher("/tactile_tools/grabbed", Bool)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
    
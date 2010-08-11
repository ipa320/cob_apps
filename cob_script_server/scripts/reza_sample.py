#!/usr/bin/python
import roslib
roslib.load_manifest('cob_script_server')
import rospy
import simple_script_server

class MyClass:
    def __init__(self):
        rospy.init_node('test_script')
        self.sss = simple_script_server.simple_script_server()

    def Initialize(self):
        rospy.loginfo("Initializing all components...")

    def GraspObject(self):
        rospy.loginfo("Grasping an object from table...")

	print "start"
	handle01=self.sss.Move("arm","home",False)
	self.sss.Move("torso","home",False)
	self.sss.Move("sdh","home",False)
	self.sss.Move("tray","down")
	handle01.wait()

        self.sss.Speak("Sent00")

	self.sss.Move("arm","pregrasp")
	self.sss.sleep(2)

	self.sss.Move("tray","up")

	self.sss.Move("sdh","cylopen")

	self.sss.Move("arm","grasp")
	self.sss.sleep(2)

	self.sss.Move("sdh","cylclosed")
	self.sss.sleep(1)

	self.sss.Move("arm","overtablet")
	self.sss.sleep(3)

	self.sss.Move("sdh","cylopen")

	self.sss.Move("arm","folded")

if __name__ == "__main__":
    SCRIPT = MyClass()
    SCRIPT.Initialize()
    SCRIPT.GraspObject()

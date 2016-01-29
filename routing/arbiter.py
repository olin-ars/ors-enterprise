import rospy
from std_msgs.msg import Int16, Float32, Bool

DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2
TEST_MODE = 3

class Arbiter:
	def __init__(self):
            self.rudder=0
            self.sail=0
            self.controlType = 0 #DEFAULT = 0
            self.debugDial = 0
            #output
            self.rudderPub = rospy.Publisher('rudder/set_point', Int16)
            self.sailPub = rospy.Publisher('sail/set_point', Int16)

            #input
            self.rudderSub = rospy.Subscriber('rc/rudder_in', Float32, self.onRCRudder) #listening to rudder_in
            self.switchSub = rospy.Subscriber('rc/switch_in', Bool, self.onRCSwitch) #listening to swith_in
            self.sailsSub = rospy.Subscriber('rc/sails_in', Float32, self.onRCSail) #listening to sails_in
            self.debugSub = rospy.Subscriber('rc/debug_dial_in', Float32, self.onDebugDial)
            #autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
            #autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)
            controlType = rospy.Subscriber('control_type', Int16,self.onControlType)
            #for further expansion

	def onRCRudder(self,data):
            self.rudder=data
            if self.controlType == RC_MODE:
                self.rudderPub.publish(data) #else ignore
	def onRCSwitch(self,data):
            if data is True:
                self.controlType = RC_MODE
            else:
                self.controlType = DEFAULT
	def onRCSail(self,data):
            self.sail=data
            if self.controlType == RC_MODE:
                self.sailPub.publish(data)
	def onAutoRudder(self,data):
            self.rudder=data
            if self.controlType == AUTO_MODE:
                self.rudderPub.publish(data)
	def onAutoSail(self,data):
            self.sail=data
            if self.controlType == AUTO_MODE:
                self.rudderPub.publish(data)
	def onControlType(self,data):
            self.controlType = data
        def onDebugDial(self,data):
            self.debugDial = data
            #currently the purpose of potentiometer is not defined?
	def spin(self):
            rospy.spin()

if __name__ == '__main__':
	arbiter = new Arbiter()
        arbiter.spin()
	# do something

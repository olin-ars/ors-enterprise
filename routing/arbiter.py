import rospy
from std_msgs.msg import Int16, Float32, Bool
DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2

class Arbiter:
	def __init__(self):
		self.RCRudder=0;
		self.RCSwitch=0;
		self.RCSail=0;
		self.ControlType = 0; #0 as RC, 1 as auto.....
		#data...
		self.rudderPub = rospy.Publisher('/ttyACM0/rudderCommands', Int16)
		self.sailPub = rospy.Publisher('/ttyACM2/sailCommands', Int16)
		self.rudderSub = rospy.Subscriber('RC_rudder_in', Float32, self.onRCRudder) #listening to rudder_in
		self.switchSub = rospy.Subscriber('RC_switch_in', Bool, self.onRCSwitch) #listening to swith_in
		self.sailsSub = rospy.Subscriber('RC_sails_in', Float32, self.onRCSail) #listening to sails_in
		#autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
		#autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)
		controlType = rospy.Subscriber('Control_type', Int16,self.onControlType)
	def onRCRudder(self,data):
		self.RCRudder=data;
		if self.ControlType == RC_MODE:
			self.rudderPub.publish(data)
			pass;
			#publish

	def onRCSwitch(self,data):
		self.RCSwitch=data;
		if self.ControlType == RC_MODE:
			#self.rudderSwitch.publish(data)
			pass;
			#publish

	def onRCSail(self,data):
		self.RCSail=data;
		if self.ControlType == RC_MODE:
			self.sailPub.publish(data)
			pass;
			#publish
	def onAutoRudder(self,data):
		pass;
	def onAutoSail(self,data):
		pass;
			
	def onControlType(self,data):
		self.ControlType = data

if __name__ == '__main__':
	arbiter = new Arbiter();
	# do something
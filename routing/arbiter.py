import rospy
from std_msgs.msg import Int16, Float32
def init():
	rudderPub = rospy.Publisher('/ttyACM0/rudderCommands', Int16)
	sailPub = rospy.Publisher('/ttyACM2/sailCommands', Int16)
	rudderSub = rospy.Subscriber('RC_rudder_in', Float32) #listening to rudder_in
	switchSub = rospy.Subscriber('RC_switch_in', Float32) #listening to swith_in
	sailsSub = rospy.Subscriber('RC_sails_in', Float32) #listening to sails_in
	#autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
	#autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)

if __name__ == '__main__':

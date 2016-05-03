#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32

class autonomousRudderPublisher:#needs a topic to read in waypoint angle from
	def __init__(self):
		rospy.init_node('rudder_publisher')
		self.rudderPub = rospy.Publisher('auto_mode/rudder/set_point', Int16)
		self.heading = 0
		def callback(angle):
			self.heading = angle
		self.waypointAngleListener = rospy.Subscriber('heading_error', Int16, callback) #needs somewhere to get waypoint
	def calculateRudderPosition(self, headingError):#calculate position rudder needs to be at to direct self at current waypoint angle, relative to bow of the boat as 0 degrees, positive is starboard and negative is port
		#currently nobody has defined rudder controls
		#rudder messages currently range from -45 to 45, assuming positive is starboard and negative is port
		turnAngle = headingError / 2 if abs(headingError) < 90 else 45 if(headingError > 0) else -45 
		return turnAngle

	def run(self):
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			rud = self.calculateRudderPosition(self.heading)
			self.rudderPub.publish(rud)
			rospy.loginfo("Sent desired rudder angle : {}".format(rud))
			rate.sleep()

if __name__ == '__main__':
	try:
		test = autonomousRudderPublisher()
		test.run()
	except rospy.ROSInterruptException:
		pass

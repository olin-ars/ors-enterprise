#!/usr/bin/env python
import rospy, time
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D

class autonomousSailPublisher:
	def __init__(self):
		rospy.init_node('sail_publisher')
		self.sailPub = rospy.Publisher('auto_mode/sail/set_point', Float32) # unsure of whether publishing to this topic actually has the sail actuator move to a given magnet position
		self.windAngle = 0
		def callback(self, data):
			self.speed = data.x
			self.windAngle = data.theta
		self.waypointAngleListener = rospy.Subscriber('relative_wind', Pose2D, callback) # placeholder for wind angle topic, positive is starboard, negative is port, angle is the direction the wind is hitting the boat from
	def calculateSailPosition(self, windAngle):# calculate sail position from wind angle
		# assuming sail positions will not be given in the dead zone and rudders will navigate away
		# sail goes from positions 0 to 6
		sailPos = (	6 if abs(windAngle) in range(155, 180) else 
					5 if abs(windAngle) in range(130, 155) else 
					4 if abs(windAngle) in range(105, 130) else 
					3 if abs(windAngle) in range(80,  105) else 
					2 if abs(windAngle) in range(55,  80)  else 
					1 if abs(windAngle) in range(30,  55)  else 0)
		
		return sailPos

	def run(self):
		
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			sailPos = self.calculateSailPosition(self.windAngle)
			self.sailPub.publish(sailPos)
			rospy.loginfo("Sent desired sail position: {}".format(sailPos))
			rate.sleep()

if __name__ == '__main__':
	try:
		test = autonomousSailPublisher()
		test.run()
	except rospy.ROSInterruptException:
		passta

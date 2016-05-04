#!/usr/bin/env python
import rospy, time
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D

# define angle cutoffs for each switch position
BORDER_ANGLES = [0, 50, 60, 70, 78, 85, 95, 110]

class autonomousSailPublisher:
	def __init__(self):
		rospy.init_node('sail_publisher')
		self.sailPub = rospy.Publisher('auto_mode/sail/set_point', Float32) # unsure of whether publishing to this topic actually has the sail actuator move to a given magnet position
		self.windAngle = 0
		self.waypointAngleListener = rospy.Subscriber('relative_wind', Pose2D, self.callback) # placeholder for wind angle topic, positive is starboard, negative is port, angle is the direction the wind is hitting the boat from
	
	def callback(self, data):
		self.speed = data.x
		self.windAngle = data.theta

	def calculateSailPosition(self, windAngle):# calculate sail position from wind angle
		# assuming sail positions will not be given in the dead zone and rudders will navigate away
		# sail goes from positions 0 to 6
		sailPos = (	6 if abs(windAngle) in range(BORDER_ANGLES[6],  BORDER_ANGLES[7]) else 
					5 if abs(windAngle) in range(BORDER_ANGLES[5],  BORDER_ANGLES[6]) else 
					4 if abs(windAngle) in range(BORDER_ANGLES[4],  BORDER_ANGLES[5]) else 
					3 if abs(windAngle) in range(BORDER_ANGLES[3],  BORDER_ANGLES[4]) else 
					2 if abs(windAngle) in range(BORDER_ANGLES[2],  BORDER_ANGLES[3])  else 
					1 if abs(windAngle) in range(BORDER_ANGLES[1],  BORDER_ANGLES[2])  else 0)
		
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

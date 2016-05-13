#!/usr/bin/env python
import rospy, time
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D

# define angle cutoffs for each switch position
BORDER_ANGLES = [0, 50, 60, 72, 85, 100, 115]

def angle_range(a):
    """ limit angles to range of -180 to 180 """
    return (a + 180) % 360 - 180

class autonomousSailPublisher:
	def __init__(self):
		rospy.init_node('sail_publisher')
		self.sailPub = rospy.Publisher('auto_mode/sail/set_point', Float32) # unsure of whether publishing to this topic actually has the sail actuator move to a given magnet position
		self.windAngle = 0
		self.waypointAngleListener = rospy.Subscriber('relative_wind', Pose2D, self.callback) # placeholder for wind angle topic, positive is starboard, negative is port, angle is the direction the wind is hitting the boat from
	
	def callback(self, data):
		self.speed = data.x
		self.windAngle = angle_range(data.theta)

	def calculateSailPosition(self, windAngle):# calculate sail position from wind angle
		# assuming sail positions will not be given in the dead zone and rudders will navigate away
		# sail goes from positions 0 to 6
		sailPos = (	6 if BORDER_ANGLES[6] < abs(windAngle) else 
					5 if BORDER_ANGLES[5] < abs(windAngle) < BORDER_ANGLES[6] else 
					4 if BORDER_ANGLES[4] < abs(windAngle) < BORDER_ANGLES[5] else 
					3 if BORDER_ANGLES[3] < abs(windAngle) < BORDER_ANGLES[4] else 
					2 if BORDER_ANGLES[2] < abs(windAngle) < BORDER_ANGLES[3]  else 
					1 if BORDER_ANGLES[1] < abs(windAngle) < BORDER_ANGLES[2]  else 0)
		
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

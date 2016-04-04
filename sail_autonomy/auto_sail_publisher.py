#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32

class autonomousSailPublisher:
	def __init__(self):
        self.sailPub = rospy.Publisher('rc_mode/sail/set_point', Int16)
        def callback(angle):
        	self.waypoint = angle
        self.waypointAngleListener = rospy.Subscriber('windAngle', Int16, callback) #placeholder for wind angle topic, positive is starboard, negative is port, angle is the direction the wind is hitting the boat from
	def calculateSailPosition(self, windAngle):#calculate sail position from wind angle
		#need to figure out how exactly sails work, this is assuming sail positions will not be given in the dead zone (estimated -45 to 45)
		sailPos = 1 if windAngle in range(-180, -135) else 2 if windAngle in range(-135, -90) else 3 if windAngle in range(-90, -45) else 4 if windAngle in range(45, 90) else 5 if windAngle in range(90, 135) else 6 if windAnglein range(90, 181)
		return sailPos

	def run(self):
		rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
        	sailPos = calculateSailPosition(self.waypoint)
        	self.sailPub.publish(sailPos)
        	rospy.loginfo("Sent desired sail position : {}".format(sailPos))
        	rate.sleep()

if __name__ == '__main__':
	try:
		test = autonomousSailPublisher()
		test.run()
	except rospy.ROSInterruptException:
		pass

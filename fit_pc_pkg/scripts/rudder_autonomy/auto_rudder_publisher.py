#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32

"""
	This class calculates the position the rudder should be at in order to
	direct the boat toward the current waypoint.
"""

class autonomousRudderPublisher:#needs a topic to read in waypoint angle from

	def __init__(self):
		""" Initialize message publisher """
		rospy.init_node('rudder_publisher')
		self.rudderPub = rospy.Publisher('auto_mode/rudder/set_point', Int16)

		""" Initialize next waypoint to be directly ahead """
		self.waypoint = 0

		""" Initialize message subscribers so appropriate functions are
			called when messages are received over ROS. """
		def callback(angle):
			# Update angle to waypoint when message is received
			self.waypoint = angle.data
		self.waypointAngleListener = rospy.Subscriber('heading_err', Int16, callback)

	""" Calculate position rudder should be at in order to move toward current waypoint.
	 	Directly off bow is 0 degrees, positive is starboard and negative is port. """
	def calculateRudderPosition(self, waypointAngle):
		#currently nobody has defined rudder controls
		#rudder messages currently range from -45 to 45, assuming positive is starboard and negative is port
		turnAngle = waypointAngle / 2 if abs(waypointAngle) < 90 else 45 if(waypoint1Angle > 0) else -45 #need information on how much exactly a rudder should be turned
		return turnAngle

	""" Loop to run through continuously while system is active """
	def run(self):
		# Tell system to sleep 10 times per second
		rate = rospy.Rate(10)  # 10 Hz
		# Perform the following loop while the system is on
		while not rospy.is_shutdown():
			# Calculate rudder position
			rud = self.calculateRudderPosition(self.waypoint)
			# Broadcast message to rest of boat with desired rudder angle
			self.rudderPub.publish(rud)
			rospy.loginfo("Sent desired rudder angle : {}".format(rud))
			rate.sleep()

""" Start the program this was called directly (not if it was imported by another file) """
if __name__ == '__main__':
	try:
		test = autonomousRudderPublisher()
		test.run()
	except rospy.ROSInterruptException:
		pass

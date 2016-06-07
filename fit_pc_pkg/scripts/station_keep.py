#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Pose2D

class Station_Keep():
	def __init__(self):
		rospy.init_node('station_keep')
		self.opModeSub = rospy.Subscriber('operating_mode', Int16, self.onOpMode)
		self.wp_checkoff = rospy.Subscriber('hit_wp', Bool, self.onWp)

		self.wp_publisher = rospy.Publisher('local_waypoints', Pose2D, queue_size=1)

		self.station_keep = False
		self.home = Pose2D()
		self.home.theta = 1

	def onOpMode(self, msg):
		if msg.data == 4:
			self.station_keep = True
		else:
			self.station_keep = False

	def onWp(self, msg):
		if msg.data and self.station_keep:
			self.timer()

	def timer(self):
		minutes = 4.5
		seconds = minutes*60
		rospy.sleep(seconds)
		self.wp_publisher.publish(self.home)

if __name__ == '__main__':
	main = Station_Keep()
	rospy.spin()

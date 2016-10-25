#!/usr/bin/env python

""" FOR TESTING AUTONOMOUS ROBOT CODE
	publishes a set of messages to reflect a boat state
	Deals with making different messages make sense to each other

	USE:
	run a python interpreter
	import this file
	make an instance of Spoof
	run appropriate methods with inputs
"""

import rospy
from geometry_msgs.msg import Pose2D

def angle_range(a):
	""" limit angles to range of -180 to 180 """
	return (a + 180)%360 - 180

def subtract_angles(a, b):
	""" subtract angles while keeping them in range -180 to 180 """
	return angle_range(a - b)

class Spoof():

	""" Initialize everything """
	def __init__(self):
		rospy.init_node('auto_test')

		""" Initialize message publishers """
		self.location_pub = rospy.Publisher('/location', Pose2D)
		self.rel_wind_pub = rospy.Publisher('/relative_wind', Pose2D)
		self.true_wind_pub = rospy.Publisher('/true_wind', Pose2D)
		self.global_wind_pub = rospy.Publisher('/global_wind', Pose2D)
		#TODO add support for changing target location

		""" Initialize current position to (0,0) """
		self.pose = [0, 0] # east, north

		""" Initialize target (waypoint) position to (1,1) and set angle """
		self.target_pose = [1, 1] # east, north
		self.angle_to_target = 45 # compass heading to target

		""" Initialize heading to be due north """
		self.heading = 0 #degrees clockwise from north

		""" Initialize wind to be due south """
		self.true_wind_angle = 0
		self.rel_wind_angle = 0
		self.global_wind_angle=0

	""" Update location of boat and wind angles as appropriate """
	def update_location(self, east=None, north=None, heading=None):
		if east != None:
			self.pose[0] = east
		if north != None:
			self.pose[1] = north
		if heading != None:
			delta = subtract_angles(heading, self.heading)
			self.heading = heading
			self.true_wind_angle -= delta
			self.rel_wind_angle -= delta
		self.pub_location()
		self.pub_rel_wind()
		self.pub_true_wind()

	""" Update the direction the wind is coming from """
	def update_global_wind(self, theta):
		delta = subtract_angles(theta, self.global_wind_angle)
		self.global_wind_angle = theta
		self.true_wind_angle += delta
		self.rel_wind_angle += delta
		self.pub_rel_wind()
		self.pub_true_wind()
		self.pub_global_wind()

	""" Publish message containing (x,y) coordinates and heading of boat """
	def pub_location(self):
		loc = Pose2D()
		loc.x = self.pose[0]
		loc.y = self.pose[1]
		loc.theta = self.heading
		self.location_pub.publish(loc)

	""" Publish message containing relative wind direction """
	def pub_rel_wind(self):
		rel_wind = self.setup_wind_pub(1, 1, self.rel_wind_angle)
		self.rel_wind_pub.publish(rel_wind)

	""" Publish message containing true wind direction """
	def pub_true_wind(self):
		true_wind = self.setup_wind_pub(1, 1, self.true_wind_angle)
		self.true_wind_pub.publish(true_wind)

	""" Publish message containing global wind direction """
	def pub_global_wind(self):
		global_wind = self.setup_wind_pub(1, 1, self.global_wind_angle)
		self.global_wind_pub.publish(global_wind)

	""" Generate a Pose2D object using the provided x, y, and theta values """
	def setup_wind_pub(self, x, y, theta):
		msg = Pose2D()
		msg.x = x
		msg.y = y
		msg.theta = theta
		return msg

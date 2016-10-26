#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Bool
from ors_2015_pkg.msg import wp_list
import math
import numpy as np

"""
	This class handles calculating the correct rudder position
	for the boat to move toward the current waypoint.
"""

#angles in world coordinates are measured clockwise from north,
# to get angle from coordinats, take atan2(deast, dnorth)
# tan(angle) = east/north

""" Define constants """
degrees = 180/math.pi
DEADZONE = 45 #deadzone off wind in degrees

""" Utility functions """
def angle_range(a):
	""" Convert a to its corresponding angle in the system
	range of -180 to 180 """
	return (a + 180)%360 - 180

def subtract_angles(a, b):
	""" subtract angles while keeping them in range -180 to 180 """
	return angle_range(a - b)

def sign(a):
	""" Returns 1 if a < 0, -1 otherwise. """
	if a == 0:
		return 1
	return a/abs(a)

class RudderThought():

	""" Initialize everything """
	def __init__(self):
		rospy.init_node('rudder_thinking')

		self.MultiTack = True

		""" Initialize coordinates of boat (standard Cartesian) """
		self.pose = [0, 0] # (0,0) (x=east, y=north)

		""" Initialize coordinates of and angle to target """
		self.target_pose = [0, 0] # (0,0) (x=east, y=north)
		self.angle_to_target = 0 # compass heading to target (degrees)

		""" Set heading and wind angle """
		self.heading = 90 #degrees clockwise from north

		self.true_wind_angle = 90
		self.rel_wind_angle = 90
		self.global_wind = 180

		""" Initialize tacking status to "not tacking" """
		self.Tacking = False
		self.tack = 1 # 1 = starboard, -1 = port

		""" Initialize message subscribers so appropriate functions are
			called when messages are received over ROS. """
		location_sub = rospy.Subscriber('location', Pose2D, self.location_callback)

		true_wind_sub = rospy.Subscriber('true_wind', Pose2D, self.true_wind_callback)

		rel_wind_sub = rospy.Subscriber('relative_wind', Pose2D, self.rel_wind_callback)

		global_wind_sub = rospy.Subscriber('global_wind', Pose2D, self.global_wind_callback)

		waypoint_sub = rospy.Subscriber('waypoints', wp_list, self.waypoints_callback)

		""" Initialize message publishers and specify message data type.
	 	queue_size here makes it so the sending of messages to registered
		recipients happens asynchronously, and the value of 1 means that
	 	it will only hold one value in the queue at a time (i.e., if one
	 	message is queued but hasn't been sent yet when a new value is
	  	published, the old queued value will be dropped and the new one
	  	will replace it). """
		self.heading_err_pub = rospy.Publisher('heading_err', Int16, queue_size=1)
		self.tacking_pub = rospy.Publisher('tacking', Bool, queue_size=1)
		self.upwind_pub = rospy.Publisher('going_upwind', Bool, queue_size=1)

	""" Loop to run through continuously while system is active """
	def run(self):
		# Initialize sleep time to 1 second (but don't actually sleep yet)
		r = rospy.Rate(1) # 1 Hz
		while not rospy.is_shutdown():
			# Calculate boat course error
			err = int(self.think())
			# Tell ye mateys! We're off course! (i.e. send the message out to all subscribed systems)
			self.heading_err_pub.publish(err)
			# Sleep for one second, then repeat loop
			r.sleep()

	def think(self):
		"""
		decide which direction the boat should go
		return appropriate error
		positive error means boat should turn clockwise
		"""
		print '\n'
		self.tacking_pub.publish(self.Tacking)
		self.upwind_pub.publish(self.is_target_upwind())

		if self.Tacking:
			print 'tack'
			err = subtract_angles(self.true_wind_angle, (DEADZONE-5)*self.tack)
			print err
			if sign(err) == self.tack:
				self.Tacking = False
			return err

		elif self.is_target_upwind():
			print 'wind', self.global_wind
			print 'target', self.angle_to_target
			print 'upwind'
			return self.wind_err()

		else:
			print 'direct'
			self.tack = sign(self.true_wind_angle)
			print self.tack
			print self.angle_to_target
			if self.is_target_accross_wind():
				self.tack *= -1
				self.Tacking = True
			return -self.target_err()


	def is_target_upwind(self):
		""" return if we should be in 'tacking mode' """
		off_wind = abs(subtract_angles(self.angle_to_target, self.global_wind))
		print 'wind', off_wind
		return off_wind <= DEADZONE + 5

	def is_target_accross_wind(self):
		""" return whether the target is accross the wind """
		wind_angle_to_target = subtract_angles(self.angle_to_target, self.global_wind)
		if abs(wind_angle_to_target) < 80:
			return self.tack*wind_angle_to_target > 0
		return False

	def wind_err(self):
		""" find the error between current heading and desired heading for going upwind """
		if self.rel_wind_angle != 0:
			self.tack = sign(self.rel_wind_angle)

		if not self.is_in_bounds():
			self.tack *= -1
			self.Tacking = True

		return subtract_angles(self.rel_wind_angle, DEADZONE*self.tack)

	def target_err(self):
		""" return angle between boat's heading and bearing to target """
		return -subtract_angles(self.angle_to_target, self.heading)

	def is_in_bounds(self):
		""" corridor code """
		if not self.MultiTack:
			return True

		""" Calculate target displacement vector relative to boat """
		wp_dx = self.pose[0] - self.target_pose[0]
		wp_dy = self.pose[1] - self.target_pose[1]
		wp_coords = np.array([wp_dx, wp_dy])

		theta = self.global_wind*math.pi/180.
		rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
		wind_coords = np.dot(rotation, wp_coords)
		return wind_coords[0]*self.tack > self.bound_func(wind_coords[1])

	def bound_func(self, y):
		""" function in form m*y+b
			m is the slope of the coridor boundary line (0 is parallel to wind)
			b is the minimum size of the corridor
		"""
		slope = 0.1
		min_width = 10
		return slope*y-min_width

	def location_callback(self, data):
		""" unpack location message and find angle to target """
		self.pose = [data.x, data.y] # position in (east, north)
		self.heading = angle_range(data.theta) #heading

		"""" get angle to target """
		delta_e = self.target_pose[0] - self.pose[0]
		delta_n = self.target_pose[1] - self.pose[1]
		self.angle_to_target = degrees*math.atan2(delta_e, delta_n)

	def true_wind_callback(self, data):
		""" get true wind angle in range -180 to 180 """
		self.true_wind_angle = angle_range(data.theta)

	def rel_wind_callback(self, data):
		""" get relative wind angle in range -180 to 180 """
		self.rel_wind_angle = angle_range(data.theta)

	def global_wind_callback(self, data):
		""" get relative wind angle in range -180 to 180 """
		self.global_wind = angle_range(data.theta)
		print 'got wind', data.theta

	def waypoints_callback(self, msg):
		""" save the next waypoint as the target location """
		wps = msg.WaypointArray
		if wps != []:
			self.target_pose = [wps[0].x, wps[0].y]

""" Start the program if this was called directly (not if it was imported by another file) """
if __name__ == '__main__':
	rud = RudderThought()
	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		rud.run()
		r.sleep()

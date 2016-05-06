#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Bool
from fit_pc_pkg.msg import wp_list
import math

#angles in world coordinates are measured clockwise from north,
# to get angle from coordinats, take atan2(deast, dnorth)
# tan(angle) = east/north

degrees = 180/math.pi
DEADZONE = 45 #deadzone off wind in degrees

def angle_range(a):
	""" limit angles to range of -180 to 180 """
	return (a + 180)%360 - 180

def subtract_angles(a, b):
	""" subtract angles while keeping them in range -180 to 180 """
	return angle_range(a - b)

def sign(a):
	""" get the sign of a number """
	if a == 0:
		return 1
	return a/abs(a)


class RudderThought():
	def __init__(self):
		rospy.init_node('rudder_thinking')

		location_sub = rospy.Subscriber('location', Pose2D, self.location_callback)

		true_wind_sub = rospy.Subscriber('true_wind', Pose2D, self.true_wind_callback)

		rel_wind_sub = rospy.Subscriber('relative_wind', Pose2D, self.rel_wind_callback)

		global_wind_sub = rospy.Subscriber('global_wind', Pose2D, self.global_wind_callback)

		waypoint_sub = rospy.Subscriber('waypoints', wp_list, self.waypoints_callback)

		self.heading_err_pub = rospy.Publisher('heading_err', Int16, queue_size=1)
		self.tacking_pub = rospy.Publisher('tacking', Bool, queue_size=1)

		self.pose = [0, 0] # east, north

		self.target_pose = [0, 0] # east, north
		self.angle_to_target = 0 # compass heading to target

		self.heading = 90 #degrees clockwise from north

		self.true_wind_angle = 90
		self.rel_wind_angle = 90
		self.global_wind=180

		self.Tacking = False
		self.tack = 1 # 1 = starboard, -1 = port

	def run(self):
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			err = int(self.think())
			self.heading_err_pub.publish(err)
			r.sleep()

	def think(self):
		""" 
		decide which direction the boat should go
		return appropriate error
		positive error means boat should turn clockwise
		"""
		print '\n'
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
			return self.target_err()
		self.tacking_pub.publish(self.Tacking)


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
		return subtract_angles(self.angle_to_target, self.heading)

	def is_in_bounds(self):
		""" corridor code """
		return True

	def location_callback(self, data):
		""" unpack location message and find angle to target """
		self.pose = [data.x, data.y] # position in (east, north)
		self.heading = data.theta #heading

		#get angle to target
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

if __name__ == '__main__':
	rud = RudderThought()
	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		rud.run()
		r.sleep()
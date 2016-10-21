#!/usr/bin/env python
import rospy
import math
from GPStoMeters import SetHome
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, Vector3
from ors_2015_pkg.msg import wp_list

'''
Handles Waypoints along with the waypoint manager.  This is the interface with the waypoint manager.
Also calculates distances between points.  Allows for skipping and clearing and removing.

Good stuff.

'''

RADIUS = 3 #acceptable wp radius in meters

def distance(point1, point2):
	""" takes in two lists or tuples representing
	points in space and returns the distance between them """
	dist_squared = 0
	for i in range(len(point1)):
		dist_squared += (point1[i]-point2[i])**2 #Calculate total distance between waypoints, could easily be used to count total distance left.
	return math.sqrt(dist_squared)

class Waypoints():
	def __init__(self):
		""" initialize the coordinates in the correct location (TODO: location input) """
		rospy.init_node('waypoint_handler') #Initialize the waypoint handler node?
		#Initialize subscribers and publishers
		wp_sub = rospy.Subscriber('raw_waypoints', Pose2D, self.new_wp_callback)
		wp_sub = rospy.Subscriber('local_waypoints', Pose2D, self.local_wp_callback)
		wp_sub = rospy.Subscriber('clear_waypoints', Bool, self.clear_wp_callback)
		wp_sub = rospy.Subscriber('rm_waypoint', Bool, self.rm_wp_callback)
		wp_sub = rospy.Subscriber('skip_waypoint', Bool, self.skip_wp_callback)
		pose_sub = rospy.Subscriber('location', Pose2D, self.pos_callback)
		self.wp_pub = rospy.Publisher('waypoints', wp_list, queue_size=1)
		self.success_pub = rospy.Publisher('hit_wp', Bool, queue_size=1)
		self.grid = SetHome() #??
		self.wp_list = [] #Waypoint list initialization
		self.wp_modes = {1: 'goto'}
		self.position = [0, 0]

	def new_wp_callback(self, msg):
		""" add a waypoint to the queue """
		north, east = self.grid.transformLocation(msg.x, msg.y)  #distance north and east that new point is
		mode = msg.theta # mode = theta
		new_wp = Vector3() #Makes new Vector3 !!!INSTEAD OF POSE
		new_wp.x = east #Sets the values
		new_wp.y = north
		new_wp.z = mode
		self.wp_list.append(new_wp) #Appends new waypoint to the current list.
		self.wp_pub.publish(self.wp_list) #Publish new list

	def local_wp_callback(self, msg):
		""" add a waypoint to the queue """
		east, north = msg.x, msg.y  #Why is this backwards?????????
		mode = msg.theta
		new_wp = Vector3()
		new_wp.x = east
		new_wp.y = north
		new_wp.z = mode
		self.wp_list.append(new_wp) #Local wp and new wp do very similar things but with the reverse of directions and no grid transform of location
		self.wp_pub.publish(self.wp_list)

	def clear_wp_callback(self, msg): #Clear list
		if msg.data:
			self.wp_list = []
			self.wp_pub.publish(self.wp_list)

	def rm_wp_callback(self, msg): #Clear last waypoint
		if msg.data:
			self.wp_list.pop()
			self.wp_pub.publish(self.wp_list)

	def skip_wp_callback(self, msg): #Clear next waypoint
		if msg.data:
			self.wp_list.pop(0)
			self.wp_pub.publish(self.wp_list)

	def pos_callback(self, msg): #Get position
		""" update the position of the boat """
		self.position = [msg.x, msg.y]

	def run(self):
		r = rospy.Rate(2) # run twice per second
		while not rospy.is_shutdown():
			if self.wp_list != []: #If list isnt empty
				wp = [self.wp_list[0].x, self.wp_list[0].y] #wp = wplist of x's and y's
				dist_to_wp = distance(self.position, wp) #calculate distance
				if dist_to_wp < RADIUS: #Make sure distance is not less than RADIUS, which is 3 meters i believe
					self.wp_list.pop(0) #If it is less than radius, skip it
					self.wp_pub.publish(self.wp_list) #Publish the update
					self.success_pub.publish(True) #And report success.
			r.sleep()

if __name__ == '__main__':
	main = Waypoints() #Run
	main.run()
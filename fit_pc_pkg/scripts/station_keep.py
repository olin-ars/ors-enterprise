#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Pose2D

'''
Handles the Station Keep operating mode.  Not exactly sure what it does.
'''

class Station_Keep():
	def __init__(self):
		rospy.init_node('station_keep') #Initialize the node station keep??
		self.opModeSub = rospy.Subscriber('operating_mode', Int16, self.onOpMode) #Subscribe to the operating mode
		self.wp_checkoff = rospy.Subscriber('hit_wp', Bool, self.onWp) #Subscribes to hit_wp which i assume to be hit waypoint.

		self.wp_publisher = rospy.Publisher('local_waypoints', Pose2D, queue_size=1) #Publishes waypoints?

		self.station_keep = False #Initilize without station keep.
		self.home = Pose2D() #Set home
		self.home.theta = 1 #set home theta

	def onOpMode(self, msg): 
	#Callback function to set operation mode. if data is 4 which corresponds to the Station Keep mode it turns station keep on
		if msg.data == 4:
			self.station_keep = True
		else:
			self.station_keep = False

	def onWp(self, msg): #if Wp topic has data to send and also the op mode is station keep, run timer
		if msg.data and self.station_keep:
			self.timer()

	def timer(self):
		#Waits 4.5 minutes and then publishes the current home location.  Don't understand the point of this.
		minutes = 4.5
		seconds = minutes*60
		rospy.sleep(seconds)
		self.wp_publisher.publish(self.home)

if __name__ == '__main__': 
	main = Station_Keep() #Run code
	rospy.spin() #don't stop code until sleeps and rates are done.

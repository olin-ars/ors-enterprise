import rospy
from GPStoMeters import SetHome
from geometry_msgs.msg import Pose2D, Vector3
from fit_pc_pkg.msg import wp_list

class Waypoints():
	def __init__(self):
		""" initialize the coordinates in the correct location (TODO: location input) """
		rospy.init_node('waypoint_handler')
		wp_sub = rospy.Subscriber('raw_waypoints', Pose2D, self.new_wp_callback)
		self.wp_pub = rospy.Publisher('waypoints', wp_list, queue_size=1)
		self.location = SetHome()
		self.wp_list = []
		self.wp_modes = {1: 'goto'}
	def new_wp_callback(self, msg):
		""" add a waypoint to the queue """
		north, east = self.location.transformLocation(msg.x, msg.y)
		mode = msg.theta
		new_wp = Vector3()
		new_wp.x = north
		new_wp.y = east
		new_wp.z = mode
		self.wp_list.append(new_wp)
		self.wp_pub.publish(self.wp_list)

if __name__ == '__main__':
	main = Waypoints()
	rospy.spin()
#! /usr/bin/python

import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D
import time

"""
    This class repeatedly prints out the current boat location, velocity, and wind
    conditions to the console.
"""

class subscriber(object):
    """docstring for subscriber"""

    location = Pose2D()
    velocity = Pose2D()
    true_wind = Pose2D()

    """ Define message handler functions """
    def loccallback(self, data):
        self.location = data

    def velcallback(self, data):
        self.velocity = data

    def wndcallback(self, data):
        self.true_wind = data

    """ Initialize everything """
    def __init__(self):
        """ Tell ROS what the name of this node is """
        rospy.init_node('sketch_gui')
        """ Register message handlers to receive appropriate messages """
        rospy.Subscriber("location", Pose2D, self.loccallback)
        rospy.Subscriber("velocity", Pose2D, self.velcallback)
        rospy.Subscriber("true_wind", Pose2D, self.wndcallback)
        """ Set module sleep rate to 20 Hz """
        r = rospy.Rate(20)

        """ While simulator is running, keep printing out boat location, boat velocity, and wind info """
        while not rospy.is_shutdown():
            print "\n"*20
            print "Location: \n", self.location, "\n\n"
            print "Velocity: \n", self.velocity, "\n\n"
            print "Wind:     \n", self.true_wind, "\n\n"
            print "\n"*5
            """ Stop execution for a little bit """
            r.sleep()

""" Create an instance of the above class (i.e., start the simulation) """
subscriber()

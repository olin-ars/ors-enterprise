#! /usr/bin/python

import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D
import time


class subscriber(object):
    """docstring for subscriber"""

    location = Pose2D()
    velocity = Pose2D()
    true_wind = Pose2D()

    def loccallback(self, data):
        self.location = data

    def velcallback(self, data):
        self.velocity = data

    def wndcallback(self, data):
        self.true_wind = data

    def __init__(self):
        rospy.init_node('sketch_gui')
        rospy.Subscriber("location", Pose2D, self.loccallback)
        rospy.Subscriber("velocity", Pose2D, self.velcallback)
        rospy.Subscriber("true_wind", Pose2D, self.wndcallback)
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            print "\n"*20
            print "Location: \n", self.location, "\n\n"
            print "Velocity: \n", self.velocity, "\n\n"
            print "Wind:     \n", self.true_wind, "\n\n"
            print "\n"*5
            r.sleep()

subscriber()

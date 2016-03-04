#! /usr/bin/python

import simulator_main as sim
import time
import math

import rospy
from std_msgs.msg import Int16, Float32


class ROShandler():
    """docstring for ROShandler"""
    def __init__(self, model):
        self.model = model
        self.registerSubscribers()

    def registerSubscribers(self):
        self.rudderSub = rospy.Subscriber('/rudder/set_point',
                                          Int16, self.onRudder)
        self.sailSub = rospy.Subscriber('/sail/set_point',
                                        Float32, self.onSail)

    def onRudder(self, msg):
        """The ROS network thinks in degrees,
        the sim things in 1/4th rotations"""
        self.model.boat1.RudderSuggestion = msg.data * 1/90.0

    def onSail(self, msg):
        """The ROS network thinks in sensors (0-6),
        the sim things in 1/4th rotations"""
        self.model.boat1.RudderSuggestion = msg.data * 1.0/6

if __name__ == '__main__':
    model = sim.WorldModel(.1, 3*math.pi/2)  # initial windspeed, windheading
    roshandler = ROShandler(model)

    r = rospy.Rate(10)
    while True:
        model.update_model()
        r.sleep()
        print model.boat1.posStr()
        # print model.boat1.forward_speed
        # print "angvel={}".format(model.boat1.angularVelocity)
        # print model.boat1.MainPos

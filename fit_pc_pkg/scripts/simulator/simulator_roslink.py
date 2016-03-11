#! /usr/bin/python

import simulator_main as sim
import math

import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D


class ROShandler():
    """docstring for ROShandler"""
    def __init__(self, model):
        rospy.init_node('simulator', anonymous=True)
        self.model = model
        self.registerSubs()
        self.registerPubs()

    def registerSubs(self):
        self.rudderSub = rospy.Subscriber('/rudder/set_point',
                                          Int16, self.onRudder)
        self.sailSub = rospy.Subscriber('/sail/set_point',
                                        Float32, self.onSail)

    def registerPubs(self):
        self.travelPub = rospy.Publisher('test/travel', Pose2D)
        self.sailPub = rospy.Publisher('sail/pos', Float32)
        self.hemispherePub = rospy.Publisher('hemisphere/position', Pose2D)

    def publish(self):
        self.travelPub.publish(Pose2D(model.boat1.xpos, model.boat1.ypos, model.boat1.heading))
        self.hemispherePub.publish(Pose2D(0, 0, model.boat1.heading*180/math.pi))
        self.sailPub.publish(Float32(model.boat1.MainPos))

    def onRudder(self, msg):
        """The ROS network thinks in degrees,
        the sim things in 1/4th rotations"""
        self.model.boat1.RudderSuggestion = msg.data * 1/90.0

    def onSail(self, msg):
        """The ROS network thinks in sensors (0-6),
        the sim things in 1/4th rotations"""
        self.model.boat1.MainSuggestion = msg.data * 1.0/6

if __name__ == '__main__':
    model = sim.WorldModel(.1, 3*math.pi/2)  # initial windspeed, windheading
    roshandler = ROShandler(model)

    r = rospy.Rate(10)
    while True:
        r.sleep()
        model.update_model()
        roshandler.publish()
        print model.boat1.posStr()
        print model.boat1.RudderPos
        # print model.boat1.forward_speed
        # print "angvel={}".format(model.boat1.angularVelocity)
        # print model.boat1.MainPos

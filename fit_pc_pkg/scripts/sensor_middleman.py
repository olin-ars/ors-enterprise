#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D, Vector3


class Arbiter:
    def __init__(self, home=(42.293441, -71.263970)):
        """home defines the center of translation. It should
        be within a few hundred km of our location. Default is the center
        of the Oval"""

        self.home = home
        self.pos = home

        self.initScalars()

        self.initSubscribers()
        self.initPublishers()

    def transformLocation(self, lat, longitude):
        return ((lat - self.home[1]) * self.latscalar,
                (longitude - self.home[2]) * self.longscalar)

    def initScalars(self):
        earth_circumference = 40.075 * 10**6  # meters
        self.latscalar = earth_circumference / 360  # meters per degree

        longdistance = math.cos(math.radians(self.home[0])) * earth_circumference  # m

        self.longscalar = longdistance / 360  # meters per degree

    def initPublishers(self):
        pass
        self.rudderPub = rospy.Publisher('rudder/set_point', Int16)
        self.sailPub = rospy.Publisher('sail/set_point', Float32)

    def initSubscribers(self):
        self.positionSub = rospy.Subscriber('/hemishphere/position', Pose2D, self.onPosition)
        pass
        self.rudderSub = rospy.Subscriber(currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in
        self.sailsSub = rospy.Subscriber(currentNamespace + '/sail/set_point', Float32, self.onSail) #listening to sails_in

    def onPosition(self, msg):
        # Incomming messages are in decimal minutes
        self.pos[1] = msg.x/60
        self.pos[2] = msg.y/60

    def onSail(self, msg):
        self.sail = msg.data
        self.sailPub.publish(msg.data)

    def onOpMode(self, msg):
        self.opMode = msg.data
        self.setupSubscribers(msg.data)

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.publish()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('arbiter', anonymous=True)
    arbiter = Arbiter()
    arbiter.run()
    # do something

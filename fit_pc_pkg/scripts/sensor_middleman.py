#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D


class Arbiter:
    def __init__(self, home=(42.293441, -71.263970)):
        """home defines the center of translation. It should
        be within a few hundred km of our location. Default is the center
        of the Oval"""

        self.home = home
        self.heading = 0.0
        self.speed = 0.0
        self.track = 0.0

        self.initScalars()

        self.pos = self.transformLocation(home[0], home[1])

        self.initSubscribers()
        self.initPublishers()

    def transformLocation(self, lat, longitude):
        return ((lat - self.home[0]) * self.latscalar,
                (longitude - self.home[1]) * self.longscalar)

    def initScalars(self):
        earth_circumference = 40.075 * 10**6  # meters
        self.latscalar = earth_circumference / 360  # meters per degree

        longdistance = math.cos(math.radians(self.home[0])) * earth_circumference  # m

        self.longscalar = longdistance / 360  # meters per degree

    def initPublishers(self):
        queue = 1

        self.posPub = rospy.Publisher('location', Pose2D, queue_size=queue)
        self.velPub = rospy.Publisher('velocity', Pose2D, queue_size=queue)

    def initSubscribers(self):
        self.positionSub = rospy.Subscriber('/hemisphere/position', Pose2D, self.onPosition)
        self.speedSub = rospy.Subscriber('/hemisphere/speed', Float32, self.onSpeed)
        self.trackSub = rospy.Subscriber('/hemisphere/track', Float32, self.onTrack)

    def onPosition(self, msg):
        # Incomming messages are in decimal minutes
        self.pos = self.transformLocation(msg.x / 60, msg.y / 60)
        self.heading = msg.theta

    def onSpeed(self, msg):
        # Incomming messages are in decimal minutes
        self.speed = msg.data

    def onTrack(self, msg):
        # Incomming messages are in degrees clockwise of North
        self.track = msg.data

    def publish(self):
        self.posPub.publish(Pose2D(self.pos[0], self.pos[1], self.heading))
        self.velPub.publish(Pose2D(self.speed, 0.0, self.track))

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

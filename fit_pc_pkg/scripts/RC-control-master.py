#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32


class RCmaster(object):
    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            rudderMsg = int(self.rudderPos)
            self.rudderPub.publish(rudderMsg)

            sailMsg = int(self.sailPos)
            self.sailPub.publish(sailMsg)

            rospy.loginfo("Sent command (Rudder, Sail)=({}, {})".format(rudderMsg, sailMsg))

            rate.sleep()

    def __init__(self):
        self.rudderPos = 0
        self.sailPos = 0  # TODO: what is the sail neutral pos?

        rospy.init_node('command_center', anonymous=True)
        # TODO: at some point, this needs to become more intelligent
        # with detecting what port the Teensy is on
        self.rudderPub = rospy.Publisher('rudder/set_point', Int16)
        self.sailPub = rospy.Publisher('sail/set_point', Int16)

        self.sailSub = rospy.Subscriber('rc/sails_in', Float32, self.update_sail)
        self.rudderSub = rospy.Subscriber('rc/rudder_in', Float32, self.update_rudder)

    def update_rudder(self, msg):
        self.rudderPos = msg.data

    def update_sail(self, msg):
        self.sailPos = msg.data


if __name__ == '__main__':
    try:
        core = RCmaster()
        core.run()
    except rospy.ROSInterruptException:
        pass

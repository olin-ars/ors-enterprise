#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32, Bool

DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2
TEST_MODE = 3
Modes = [DEFAULT, RC_MODE, AUTO_MODE, TEST_MODE]


class Arbiter:
    def __init__(self):
        self.rudder = 0
        self.sail = 0
        self.opMode = 0 # DEFAULT = 0
        self.debugDial = 0
        #output
        self.rudderPub = rospy.Publisher('rudder/set_point', Int16)
        self.sailPub = rospy.Publisher('sail/set_point', Float32)

        self.rudderSub = None
        #autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
        #autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)
        self.opModeSub = rospy.Subscriber('operating_mode', Int16,self.onOpMode)

        self.switchSub = rospy.Subscriber('rc/switch_in', Bool, self.onRCSwitch)
        #for further expansion
        self.mode = RC_MODE
        self.setupSubscribers()
        self.RC_OVERRIDE = False

    def setupSubscribers(self):
        namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:"auto_mode", TEST_MODE:"test_mode"}
        currentNamespace = namespaces[self.mode]
        if self.RC_OVERRIDE:
            currentNamespace = "rc_mode"

        #input
        try:
            self.rudderSub.unregister();
            self.sailsSub.unregister();
        except:
            pass

        if not currentNamespace:
            return

        self.rudderSub = rospy.Subscriber(currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in
        self.sailsSub = rospy.Subscriber(currentNamespace + '/sail/set_point', Float32, self.onSail) #listening to sails_in
    
    def onRudder(self,msg):
        self.rudder=msg.data
        self.rudderPub.publish(msg.data)

    def onRCSwitch(self,msg):
        if msg.data and not self.RC_OVERRIDE:
            self.RC_OVERRIDE = True
            self.setupSubscribers()
        elif self.RC_OVERRIDE and not msg.data
            self.RC_OVERRIDE = False
            self.setupSubscribers()

    def onSail(self,msg):
        self.sail=msg.data
        self.sailPub.publish(msg.data)

    def onOpMode(self,msg):
        self.mode = msg.data
        self.setupSubscribers()

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('arbiter', anonymous=True)
    arbiter = Arbiter()
    arbiter.spin()
    # do something

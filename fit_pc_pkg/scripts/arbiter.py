#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32, Bool

DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2
SEMI_AUTO = 3
TEST_MODE = 4
Modes = [DEFAULT, RC_MODE, AUTO_MODE, SEMI_AUTO, TEST_MODE]


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

    def setupSubscribers(self):
        rud_namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:"auto_mode", SEMI_AUTO:"rc_mode", TEST_MODE:"test_mode"}
        sail_namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:"auto_mode", SEMI_AUTO:"auto_mode", TEST_MODE:"test_mode"}
        rud_currentNamespace = rud_namespaces[self.mode]
        sail_currentNamespace = sail_namespaces[self.mode]

        #input
        try:
            self.rudderSub.unregister();
            self.sailsSub.unregister();
        except:
            pass

        if not rud_currentNamespace or not sail_currentNamespace:
            return

        self.rudderSub = rospy.Subscriber(rud_currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in
        self.sailsSub = rospy.Subscriber(sail_currentNamespace + '/sail/set_point', Float32, self.onSail) #listening to sails_in
    
    def onRudder(self,msg):
        self.rudder=msg.data
        self.rudderPub.publish(msg.data)

    def onRCSwitch(self,msg):
        if msg.data and self.mode != RC_MODE:
            self.mode = RC_MODE
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

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32, Bool

DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2
TEST_MODE = 3
Modes = [DEFAULT,RC_MODE,AUTO_MODE,TEST_MODE]

class Arbiter:
    def __init__(self):
        self.rudder=0
        self.sail=0
        self.opMode = 0 #DEFAULT = 0
        self.debugDial = 0
        #output
        self.rudderPub = rospy.Publisher('rudder/set_point', Int16)
        self.sailPub = rospy.Publisher('sail/set_point', Int16)

        self.rudderSub = None
        #autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
        #autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)
        self.opModeSub = rospy.Subscriber('operating_mode', Int16,self.onOpMode)
        #for further expansion
        self.setupSubscribers(RC_MODE)

    def setupSubscribers(self, mode):
        namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:None, TEST_MODE:"test_mode"}
        currentNamespace = namespaces[mode]

        #input
        if self.rudderSub:
            self.rudderSub.unregister();
            self.sailsSub.unregister();

        if not currentNamespace:
            return

        self.rudderSub = rospy.Subscriber(currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in
        self.sailsSub = rospy.Subscriber(currentNamespace + '/sail/set_point', Int16, self.onSail) #listening to sails_in
    
    def onRudder(self,msg):
        self.rudder=msg.data
        self.rudderPub.publish(msg.data)

#    def onRCSwitch(self,msg):
#        if msg.data is True:
#            self.controlType = RC_MODE
#        else:
#            self.controlType = DEFAULT
#
    def onSail(self,msg):
        self.sail=msg.data
        self.sailPub.publish(msg.data)

    def onOpMode(self,msg):
        self.opMode = msg.data
        self.setupSubscribers(msg.data)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('arbiter', anonymous=True)
    arbiter = Arbiter()
    arbiter.spin()
    # do something

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
        setupSubscribers(self,DEFAULT)

    def setupSubscribers(self, mode):
        namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:None, TEST_MODE:"test_mode"}
        currentNamespace = namespaces[mode]

        #input
        if rudderSub:
            self.rudderSub.unregister();
            self.sailsSub.unregister();

        if not currentNamespace:
            return

        self.rudderSub = rospy.Subscriber(currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in
        self.sailsSub = rospy.Subscriber(currentNamespace + '/sail/set_point', Int16, self.onSail) #listening to sails_in
    
    def onRudder(self,data):
        self.rudder=data
        self.rudderPub.publish(data)

#    def onRCSwitch(self,data):
#        if data is True:
#            self.controlType = RC_MODE
#        else:
#            self.controlType = DEFAULT
#
    def onSail(self,data):
        self.sail=data
        self.sailPub.publish(data)

    def onOpMode(self,data):
        self.opMode = data
        setupSubscribers(self,data)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
	arbiter = new Arbiter()
        arbiter.spin()
	# do something

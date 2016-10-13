#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32, Bool

#Setting a list of numbers that can be called as modes.

DEFAULT = 0
RC_MODE = 1
AUTO_MODE = 2
SEMI_AUTO = 3
STATION_KEEP = 4
TEST_MODE = 5
Modes = [DEFAULT, RC_MODE, AUTO_MODE, SEMI_AUTO, STATION_KEEP, TEST_MODE] #This variable isn't used


class Arbiter:
    def __init__(self): #Initializing everything to zero and setting variables
        self.rudder = 0  #Rudder Angle?
        self.sail = 0  #Sail Angle?
        self.opMode = 0  # DEFAULT = 0 ??? Opening Mode?
        self.debugDial = 0  #???
        #output
        self.rudderPub = rospy.Publisher('rudder/set_point', Int16) #Make a publisher that tells the current ruder position?
        self.sailPub = rospy.Publisher('sail/set_point', Float32)  #make a publisher that tells the current sail position?

        self.rudderSub = None  #No rudder Subscriber
        #Also no Sail subscriber
        #autoRudderSub = rospy.Subscriber('auto_rudder_in', Float32)
        #autoSailsSub = rospy.Subscriber('auto_sails_in', Float32)
        self.opModeSub = rospy.Subscriber('operating_mode', Int16,self.onOpMode) #Set a subscriber to read from Operating_mode???

        self.switchSub = rospy.Subscriber('rc/switch_in', Bool, self.onRCSwitch) #Set a subscriber to read whether the RC switch is on?
        #for further expansion
        self.RC_OVERRIDE = False  #Whether the RC has overrided (Probably?)
        self.mode = RC_MODE  #Initialize in RC_Mode
        self.setupSubscribers() #Run the Subscriber Setup

    def setupSubscribers(self): #Subscriber function, part of setup
    	#USED TO CHANGE THE SUBSCRIBER, aka what the code is listening to for information.
        rud_namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:"auto_mode", SEMI_AUTO:"rc_mode", TEST_MODE:"test_mode", STATION_KEEP:"auto_mode"} #List of rudder behaviours modes (Corresponding with the different settings) IMPORTANT NOTE!!!! None is essentially NULL in python!
        sail_namespaces = {DEFAULT: None, RC_MODE: "rc_mode", AUTO_MODE:"auto_mode", SEMI_AUTO:"auto_mode", TEST_MODE:"test_mode", STATION_KEEP:"auto_mode"} #List of sail behaviours modes (Corresponding with the different settings) Sail is automatic unless in full RC mode.
        rud_currentNamespace = rud_namespaces[self.mode] #Set Rudder Mode
        sail_currentNamespace = sail_namespaces[self.mode] #Set Sail Mode 
        if self.RC_OVERRIDE:
            rud_currentNamespace = "rc_mode" #If Overridden, put rudder into RC Mode
            sail_currentNamespace = "rc_mode" #If Overridden, put sail into RC Mode

        #input
        try:
            self.rudderSub.unregister(); #Unregisters from the Rudder Subscriber to reinitialize it later
            self.sailsSub.unregister();  #Unregisters from the Sail Subscriber to reinitialize it later
        except:
            pass

        if not rud_currentNamespace or not sail_currentNamespace: #If something ridiculous happened and eighter rudder or sail type isnt set, then just DONT DO ANYTHING.  (This also happens if it is default because default = None)
            return

        self.rudderSub = rospy.Subscriber(rud_currentNamespace + '/rudder/set_point', Int16, self.onRudder) #listening to rudder_in, Initializes rudder subscriber as new mode (rud_namespaces)
        self.sailsSub = rospy.Subscriber(sail_currentNamespace + '/sail/set_point', Float32, self.onSail) #listening to sails_in, Initializes sail subscriber as new mode (sail_namespaces)
    
    def onRudder(self,msg):
        self.rudder=msg.data  #Recieves the data from RudderSub, and stores it in rudder, 
        self.rudderPub.publish(msg.data)  #publishes new data to rudderPub

    def onRCSwitch(self,msg): #Receives the data from onRCSwitchSub  MSG.DATA = THE SUBSCRIBED INFORMATION
        if msg.data and not self.RC_OVERRIDE: #If there is data and there isn't an overRide
            self.RC_OVERRIDE = True #Override
            self.setupSubscribers() #Reset Subscribers and mode
        elif self.RC_OVERRIDE and not msg.data: # If there isn't data, or data is None / False and OverRide is True
            self.RC_OVERRIDE = False #Unoverride
            self.setupSubscribers() #Reset Subscribers and mode

    def onSail(self,msg): 
        self.sail=msg.data #Recieves the data from sailsSub, and stores it in sail
        self.sailPub.publish(msg.data) #publishes new data to sailpub

    def onOpMode(self,msg):
        self.mode = msg.data #Receives the data from onOpModeSub
        self.setupSubscribers() #Reset Subscribers.  HOWEVER I don't think this currently doesn't do anything outside of in the initialization and doesn't really change anything for setupSubscribers()

    def spin(self):
        rospy.spin() #Stop python from closing while the node is still publishing or subscribing

if __name__ == '__main__': #If this code is the main code:
    rospy.init_node('arbiter', anonymous=True) #Something about initializing a node called arbiter that is anonymous???
    arbiter = Arbiter() #Instantiate the Arbiter class (all the code above this)
    arbiter.spin() #perform spin and make sure python doesn't suddenly close.
    # do something 

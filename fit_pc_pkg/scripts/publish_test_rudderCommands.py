#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32

import random

'''Publisher / Subscriber Test for the Rudder.'''

def talker():

    global potPos; 
    potPos = 180.0;

    pub = rospy.Publisher('test_mode/rudder/set_point', Int16) #create publisher
    def callback(msg): 
        global potPos #TODO: this is ugly.
        potPos = msg.data
    pot_sub = rospy.Subscriber('rc/debug_dial_in', Float32, callback) #Create subscriber to read and set potPos
    rospy.init_node('command_center', anonymous=True) #Initialize the rosNode command center???


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        command = int(potPos + 180); # add 180 to potPos???
        rospy.loginfo("Sent command {}".format(command)) #Log sent info
        pub.publish(command) #Publish the command
        rate.sleep() #Sleep for rate

if __name__ == '__main__':
    try:
         talker() #On running this script, initialize talker.
    except rospy.ROSInterruptException:
        pass
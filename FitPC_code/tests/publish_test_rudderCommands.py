#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32

import random

def talker():

    global potPos;
    potPos = 180.0;

    pub = rospy.Publisher('/ttyACM0/rudderCommands', Int16)
    def callback(msg): 
        global potPos #TODO: this is ugly.
        print 'callback run', potPos
        potPos = msg.data
    pot_sub = rospy.Subscriber('/ttyACM1/potentiometer', Float32, callback)
    rospy.init_node('talker', anonymous=True)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        command = int(potPos + 180);
        rospy.loginfo("Sent command {}".format(command))
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
         talker()
    except rospy.ROSInterruptException:
        pass
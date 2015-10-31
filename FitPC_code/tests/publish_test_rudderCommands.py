#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16


def talker():
    pub = rospy.Publisher('rudderCommands', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.2) # .2hz
    while not rospy.is_shutdown():
        command = 100;
        rospy.loginfo("Sent command {}".format(command))
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
         talker()
    except rospy.ROSInterruptException:
        pass
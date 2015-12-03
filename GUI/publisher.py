#!usr/bin/env python
import rospy
import math
from std_msgs.msg import *

def ORSBoat():
    pub = rospy.Publisher('sample_msg',Float32,queue_size=10);
    rospy.init_node('talker',anonymous=True);
    rate = rospy.Rate(10);
    theta = 0;
    while not rospy.is_shutdown():
        pub.publish(math.radians(theta));
        rate.sleep();
        theta += 1;
        if theta >= 360:
            theta = 0;
if __name__ == '__main__':
    try:
        ORSBoat();
    except rospy.ROSInterruptException:
        pass;


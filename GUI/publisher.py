#!usr/bin/env python
import rospy
import math
import random
from std_msgs.msg import *
from geometry_msgs.msg import Pose2D, Vector3

def ORSBoat():
    msgs = {'rudder/pos' : Int16,
            'rudder/set_point' : Int16,
            'sail/pos' : Float32,
            'sail/set_point' : Float32,
            'hemisphere/position' : Pose2D,
            'hemisphere/speed' : Float32,
            'hemisphere/track' : Float32,
            'hemisphere/attitude' : Vector3,
            #'hemisphere/magnetic_direction' : Float32,
            #'hemisphere/status' : Bool,
            #'hemisphere/errors' : String,
            #'hemisphere/fullmsg' : String,
        }
    pubs = {}
    for msg,msg_type in msgs.iteritems():
        pubs[msg] = rospy.Publisher(msg,msg_type,queue_size=10)
    
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for p,m in msgs.iteritems():
            if p == 'hemisphere/attitude':
                pubs[p].publish(1,2,3)
            elif p == 'hemisphere/position':
                pubs[p].publish(42.28,-71.24,3)
            else:
                v = int(random.random()*10)
                pubs[p].publish(v)
        rate.sleep()
if __name__ == '__main__':
    try:
        ORSBoat()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python
#must have pyserial installed, can do through terminal
#pip install pyserial
import rospy
import serial
from std_msgs.msg import Int16, Float32, Bool, String
from geometry_msgs.msg import Pose2D

class hemisphere_parser:
    def __init__(self):
        self.init_serial()
        self.init_ros_node()
        self.init_ros_msgs()

    def init_ros_node(self):
        #ros stuff
        rospy.init_node('hemisphere')
        #time 
        #self.hemisphereTimePub = rospy.Publisher('hemisphere/time', Int16, queue_size=5) #time as hhmmss
        #status (void or active)
        self.hemisphereStatusPub = rospy.Publisher('hemisphere/status', Bool, queue_size=5) #A=active or V=void
        #position in GPS coordinates and heading
        #TODO: get heading correct
        self.hemispherePosNumbersPub = rospy.Publisher('hemisphere/position_numbers', Pose2D, queue_size=10) #lattitude, longtitude, compass heading
        #speed in ?
        self.hemisphereSpeedPub = rospy.Publisher('hemisphere/speed', Float32, queue_size=10)
        #angle ????
        self.hemisphereAnglePub = rospy.Publisher('hemisphere/angle', Float32, queue_size=10)
        #magnetic varriation
        #self.hemisphereMagenticVariationPub = rospy.Publisher('hemisphere/magnetic_variation', Float32, queue_size=2)
        #compass heading? ????
        self.hemisphereMagneticDirectionPub = rospy.Publisher('hemisphere/magnetic_direction', Float32, queue_size=2) #NSEW

    def init_ros_msgs(self):
        self.position = Pose2D
        self.status = False
        self.speed = 0
        self.angle = 0
        self.mag_dir = 0

    def init_serial(self):
        #serial reader to receive input via USB
        self.ser = serial.Serial()
        self.ser.port = "/dev/ttyUSB0"
        self.ser.baudrate = 19200
        self.ser.open()

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            hemisphereMsg = self.ser.readline() #have been unable to actually test reading in
            #If only using a single publisher
            '''self.hemisperePub.publish(hemisphereMsg)
            rospy.loginfo("Sent hemisphere data {}".format(hemisphereMsg))
            '''
            #Using multiple publishers
            print hemisphereMsg
            messageArray = hemisphereMsg.split(',')
            print messageArray

            if messageArray[0] == '$GPRMC':
                self.parse_GPRMC(messageArray)
                self.publish_GPRMC()
            elif messageArray[1] == '$PASHR':
                self.parse_PASHR()
                self.publish_PASHR()
            ### Add code for more messages here ###
            
            rate.sleep()
    def parse_GPRMC(self, msg):
        if(msg[2] == 'A' or msg[2]==''):
            self.status = True
        else:
            self.status = False
        self.position.x = float(msg[3])
        self.position.y = float(msg[5])
        if msg[4] == 'S':
            self.position.x *= -1
        if msg[6] == 'W':
            self.position.y *= -1
        self.speed = float(msg[7])
        self.angle = float(msg[8])
        #self.mag_var = msg[10]

    def publish_GPRMC(self):
        self.hemisphereStatusPub.publish(self.status)
        self.hemispherePosNumbersPub.publish(self.position)
        self.hemisphereSpeedPub.publish(self.speed)
        self.hemisphereAnglePub.publish(self.angle)
        #self.hemisphereMagenticVariationPub.publish(self.mag_var)

    def parse_PASHR(self, msg):
        pass

    def publish_PASHR(self):
        pass

if __name__ == '__main__':
    try:
        core = hemisphere_parser()
        core.run()
    except rospy.ROSInterruptException:
        pass
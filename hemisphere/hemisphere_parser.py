#!/usr/bin/env python
#must have pyserial installed, can do through terminal
#pip install pyserial
import rospy
import serial
from std_msgs.msg import Int16, Float32, Bool, String
from geometry_msgs.msg import Pose2D, Vector3

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
        self.StatusPub = rospy.Publisher('hemisphere/status', Bool, queue_size=5) #A=active or V=void
        #position in GPS coordinates and heading (decimal minutes, + = N and E, - = S and W)
        self.PositionPub = rospy.Publisher('hemisphere/position', Pose2D, queue_size=10) #lattitude, longtitude, compass heading
        #speed in Knots
        self.SpeedPub = rospy.Publisher('hemisphere/speed', Float32, queue_size=10)
        #track made good (relative to true north)
        self.TrackPub = rospy.Publisher('hemisphere/track', Float32, queue_size=10)
        #attitude (roll, pitch and heave)
        self.AttitudePub = rospy.Publisher('hemisphere/attitude', Vector3, queue_size = 5)
        #magnetic varriation
        #self.hemisphereMagenticVariationPub = rospy.Publisher('hemisphere/magnetic_variation', Float32, queue_size=2)
        #compass heading? ????
        self.hemisphereMagneticDirectionPub = rospy.Publisher('hemisphere/magnetic_direction', Float32, queue_size=2) #NSEW
        #error strings we want to see
        self.ErrorPub = rospy.Publisher('hemisphere/errors', String, queue_size=1)
        #full message for debug
        self.FullMsgPub = rospy.Publisher('hemisphere/fullmsg', String, queue_size=1)

    def init_ros_msgs(self):
        self.position = Pose2D()
        self.status = False
        self.speed = 0
        self.track = 0
        self.mag_dir = 0
        self.attitude = Vector3()

    def init_serial(self):
        #serial reader to receive input via USB
        self.ser = serial.Serial()
        self.ser.port = "/dev/ttyUSB0"
        self.ser.baudrate = 19200
        self.ser.open()

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            #read data from hemisphere
            try:
                hemisphereMsg = self.ser.readline()
            except rospy.ROSInterruptException:
                self.ErrorPub.publish('no GPS') #publish an error if no data is available
                self.status = False
            self.FullMsgPub.publish(hemisphereMsg) #publish full msg
            
            #parse message:
            messageArray = hemisphereMsg.split(',')

            if messageArray[0] == '$GPRMC': #parse GPRMC message
                self.parse_GPRMC(messageArray)
                self.publish_GPRMC()

            elif messageArray[0] == '$PASHR': #parse PASHR message
                self.parse_PASHR(messageArray)
                self.publish_PASHR()
                self.ser.flushInput()

            ### Add code for more messages here ###
            rate.sleep()

    def parse_GPRMC(self, msg):
        """ Parse the GPRMC message from the hemisphere
            Gives GPS position, speed, track """
        if(msg[2] == 'A'): #A = valid, V = void
            self.status = True
            self.position.x = float(msg[3]) #latitude (decimal minutes)
            self.position.y = float(msg[5]) #longitude
            #convert S and W to negatives for lat lon
            if msg[4] == 'S':
                self.position.x *= -1
            if msg[6] == 'W':
                self.position.y *= -1
            self.speed = float(msg[7])
            self.track = float(msg[8])
            #self.mag_var = msg[10]
        else:
            self.status = False

    def publish_GPRMC(self):
        self.StatusPub.publish(self.status)
        self.PositionPub.publish(self.position)
        self.SpeedPub.publish(self.speed)
        self.TrackPub.publish(self.track)
        #self.hemisphereMagenticVariationPub.publish(self.mag_var)

    def parse_PASHR(self, msg):
        """ Parse PASHR message from hemisphere
            give us true heading, roll, pitch and heave """
        if msg[3] == 'T': #true heading (empty if no data)
            self.position.theta = msg[2] #true heading
            self.attitude.x = float(msg[7]) #roll
            self.attitude.y = float(msg[8]) #pitch
            self.attitude.z = float(msg[6]) #heave

    def publish_PASHR(self):
        self.PositionPub.publish(self.position)
        self.AttitudePub.publish(self.attitude)

if __name__ == '__main__':
    try:
        core = hemisphere_parser()
        core.run()
    except rospy.ROSInterruptException:
        print 'ahh'
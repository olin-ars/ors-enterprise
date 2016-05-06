#!/usr/bin/env python
#must have pyserial installed, can do through terminal
#pip install pyserial
import rospy
import serial
import sys
from std_msgs.msg import Int16, Float32, Bool, String
from geometry_msgs.msg import Pose2D, Vector3

class airmar_parser:
    def __init__(self, port = "/dev/ttyUSB0"):
        self.port = port
        self.init_serial()
        self.init_ros_node()
        self.init_ros_msgs()

    def init_ros_node(self):
        #ros stuff
        rospy.init_node('airmar')
        #status (void or active)
        self.StatusPub = rospy.Publisher('airmar/status', Bool, queue_size=5) #A=active or V=void
        #position in GPS coordinates and heading (decimal minutes, + = N and E, - = S and W)
        self.PositionPub = rospy.Publisher('airmar/position', Pose2D, queue_size=10) #lattitude, longtitude, compass heading
        #speed in Knots
        self.SpeedPub = rospy.Publisher('airmar/speed', Float32, queue_size=10)
        #track made good (relative to true north)
        self.TrackPub = rospy.Publisher('airmar/track', Float32, queue_size=10)
        #relative wind speed and direction
        self.RelWindPub = rospy.Publisher('airmar/relative_wind', Pose2D, queue_size = 5)
        #true wind speed and direction
        self.TrueWindPub = rospy.Publisher('airmar/true_wind', Pose2D, queue_size = 5)
        #compass heading? ????
        self.MagneticDirectionPub = rospy.Publisher('airmar/magnetic_direction', Float32, queue_size=2) #NSEW
        #error strings we want to see
        self.ErrorPub = rospy.Publisher('airmar/errors', String, queue_size=1)
        #full message for debug
        self.FullMsgPub = rospy.Publisher('airmar/fullmsg', String, queue_size=1)

    def init_ros_msgs(self):
        self.position = Pose2D()
        self.status = False
        self.speed = 0
        self.track = 0
        self.mag_dir = 0
        self.rel_wind = Pose2D()
        self.true_wind = Pose2D()

    def init_serial(self):
        #serial reader to receive input via USB
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = 4800
        self.ser.open()

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            #read data from hemisphere
            try:
                Msg = self.ser.readline()
            except rospy.ROSInterruptException:
                self.ErrorPub.publish('no airmar') #publish an error if no data is available
                self.status = False
            self.FullMsgPub.publish(Msg) #publish full msg
            
            #parse message:
            messageArray = Msg.split(',')

            if messageArray[0] == '$WIMWV': #parse GPRMC message
                self.parse_WIMWV(messageArray)
                self.publish_WIMWV()

            elif messageArray[0] == '$HCHDT': #parse PASHR message
                self.parse_HCHDT(messageArray)
                self.publish_HCHDT()

            elif messageArray[0] == '$GPGLL': #parse PASHR message
                self.parse_GPGLL(messageArray)
                self.publish_GPGLL()

            elif messageArray[0] == '$GPVTG': #parse PASHR message
                self.parse_GPVTG(messageArray)
                self.publish_GPVTG()
                self.ser.flushInput()
            ### Add code for more messages here ###
            rate.sleep()

    def parse_WIMWV(self, msg):
        """ Parse the WIMWV message from the airmar
            Gives wind speed and angle """
        if msg[-1][0] == 'A': #A = valid, V = void
            self.status = True
            if(msg[2] == 'R'): #relative wind
                self.rel_wind.theta = float(msg[1]) #wind direction (knots?)
                self.rel_wind.x = float(msg[3]) #wind speed
            else: #msg[2] == 'T' for true wind
                self.true_wind.theta = float(msg[1]) #wind direction (knots?)
                self.true_wind.x = float(msg[3]) #wind speed
        else:
            self.status = False

    def publish_WIMWV(self):
        self.RelWindPub.publish(self.rel_wind)
        self.TrueWindPub.publish(self.true_wind)

    def parse_HCHDT(self, msg):
        """ Parse the WIMWV message from the airmar
            Gives wind speed and angle """
        self.position.theta = float(msg[1])

    def publish_HCHDT(self):
        self.PositionPub.publish(self.position)

    def parse_GPGLL(self, msg):
        """ Parse the WIMWV message from the airmar
            Gives wind speed and angle """

        def convert_angle(string, longitude=False):
        if not longitude:
            string = '0' + string
        if not string:
            return 0
        degreepart = int(string[:3])
        minutepart = float(string[3:])
        return degreepart + minutepart/60

        if msg[-1][0] == 'A': #A = valid, V = void
            self.position.x = convert_angle(msg[1]) #latitude (decimal minutes)
            self.position.y = convert_angle(msg[3]) #longitude
            #convert S and W to negatives for lat lon
            if msg[2] == 'S':
                self.position.x *= -1
            if msg[4] == 'W':
                self.position.y *= -1
        else:
            self.status = False

    def publish_GPGLL(self):
        self.StatusPub.publish(self.status)
        self.PositionPub.publish(self.position)

    def parse_GPVTG(self, msg):
        """ Parse GPVTGG message from hemisphere
            give us true heading, roll, pitch and heave """
        if msg[1] != '':
            self.track = float(msg[1]) #true heading
            self.speed = float(msg[5]) #speed (knots)

    def publish_GPVTG(self):
        self.TrackPub.publish(self.track)
        self.SpeedPub.publish(self.speed)

if __name__ == '__main__':
    try:
        port = sys.argv[1]
        core = airmar_parser(port)
        core.run()
    except rospy.ROSInterruptException:
        print 'ahh'

"""
HCHDT - heading, hopefully true (vs. magnetic)
WIMWV - wind speed and angle
    R - "relative" to boat heading - no extra math
    T - true - still relative to boat, but corrected for boat's speed
WIMWD - wind speed and angle relative to north
    <1> - true north (<2> should be 'T')
    <3> - magnetic north (<4> should be 'M')
    <5> - wind speed knots (<6> should be 'N')
    <7> - wind speed m/s (don't use this, we are using knots)
GPGLL - GPS
GPVTG - speed over ground (SOG), course over ground (COG)

$PAMTC,EN,MWD - enable MWD command

$YXXDR,A,86.0,D,PTCH,A,-85.2,D,ROLL*71
$GPVTG,,,,,,,,,N*30
$HCHDT,115.1,T*2D
$WIMWV,250.0,R,0.7,N,A*23
$GPGLL,,,,,,V,N*64
$GPZDA,,,,*48
$WIMWV,,T,,,V*7C
$GPVTG,,,,,,,,,N*30
"""

#!/usr/bin/env python
#must have pyserial installed, can do through terminal
#pip install pyserial
import rospy
import serial
from std_msgs.msg import Int16, Float32, Boo

class hemisphere_parser:
	def __init__(self):
		#This if if we want the hemisphere data to be serparated then published to corresponding topics to be handled within this parser module
		self.hemisphereFixPub = rospy.Publisher('hemisphere/fix', Int16) #time as hhmmss
		self.hemisphereStatusPub = rospy.Publisher('hemisphere/status', Int16) #A=active or V=void
		self.hemispherePosNumbersPub = rospy.Publisher('hemisphere/position_numbers', Float32) #lattitude, longtitude
		self.hemispherePosDirectionPub = rospy.Publisher('hemisphere/position_direction', String) #lattitude, longtitude, with values of N, S, E, or W for compass directions
		self.hemisphereSpeedPub = rospy.Publisher('hemisphere/speed', Float32)
		self.hemisphereAnglePub = rospy.Publisher('hemisphere/angle', Float32)
		self.hemisphereDatePub = rospy.Publisher('hemisphere/date', Int16) #ddmmyy
		self.hemisphereMagenticVariationPub = rospy.Publisher('hemisphere/magnetic_variation', Float32)
		self.hemisphereMagneticDirectionPub = rospy.Publisher('hemisphere/magnetic_direction', String) #NSEW
		self.hemisphereChecksumPub = rospy.Publisher('hemisphere/checksum', String) #checksum, always begins with *

		#This is if we want the the entire GPRMC message line to be output as a single String
		#self.hemispherePub = rospy.Publisher('hemisphere', String)

		#serial reader to receive input via USB
		self.ser = serial.Serial()
		self.ser.port = "dev/ttyUSB0"
		self.ser.baudrate = 19200
		self.ser.open()
	def run(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
        	hemisphereMsg = self.ser.read() #have been unable to actually test reading in
        	 #If only using a single publisher
        	 '''self.hemisperePub.publish(hemisphereMsg)
        	 rospy.loginfo("Sent hemisphere data {}".format(hemisphereMsg))
        	 '''
        	 #Using multiple publishers
        	 messageArray = hemisphereMsg.split(',')
        	 self.hemisphereFixPub.publish(messageArray[1])
        	 self.hemisphereStatusPub.publish(messageArray[2])
        	 self.hemispherePosNumbersPub.publish(messageArray[3], messageArray[5])
        	 self.hemisphereMagneticDirectionPub.publish(messageArray[4], messageArray[6])
        	 self.hemisphereSpeedPub.publish(messageArray[7])
        	 self.hemisphereAnglePub.publish(messageArray[8])
        	 self.hemisphereDatePub.publish(messageArray[9])
        	 self.hemisphereMagenticVariationPub.publish(messageArray[10])
        	 self.hemisphereMagneticDirectionPub.publish(messageArray[11].split('*')[0])
        	 self.hemisphereChecksumPub.publish(messageArray[11].split('*')[1])
        	 rate.sleep()
if __name__ == '__main__':
    try:
        core = hemisphere_parser()
        core.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
#import rospy
#from std_msgs.msg import Int16

'''
A way to communicate os commands through python to connect everything
Allows the connection of other shells, and connecting

'''

import subprocess
import os.path
from time import sleep
activeShells = {};

def getIP():
	return subprocess.check_output(['hostname','--all-ip-addresses']).strip() #Parse the IP of all connected objects

def run_rosserial(port = '/dev/ttyACM0'):
	#Connect to port /dev/ttyACM0 unless it doesn't exist
	if (not os.path.exists(port)): 
		return None;
	"""
	command = 'export ROS_IP=$(hostname --all-ip-addresses);\    #????
	echo ROS_IP = "$ROS_IP";\ """                                #????

	#perform some two commands to open port, and specifically receive nodal information from the ports.
	command = 'export ROS_NAMESPACE={ns};\
	rosrun rosserial_python serial_node.py {port}'.format(ns=port.split('/')[-1] ,port=port)

	p = subprocess.Popen(command, shell=True)
	return p

def refreshShells():
	for k in activeShells.keys(): #Check all active shell keys
		if activeShells[k].poll() is not None: #For each active shell do something?? .poll()?
			print 'ROSserial process for {} has died'.format(k)
			# The shell has finished
			del activeShells[k]
			
	for i in range(4): # Check 4 folders in dev 
		path = '/dev/ttyACM{}'.format(i) # set path to one of the folders

		if path in activeShells: #Make sure path is in active shell
			print 'ROSserial running on {}'.format(path)
			continue
		else:
			p = run_rosserial(path) #If not there run the shell
			if p:
				activeShells[path] = p #If p worked 
				print 'Started ROSserial for {}'.format(path)
			else: #If p doesnt work
				print 'Unable to start ROSserial for {}'.format(path)
	print ''


def killAllShells(): #Kill shells
	print '\nShutting down all active shells' 
	for k in activeShells:
		activeShells[k].terminate()
	sleep(2)

def main():
	#Run shells, currently commented out.
	#activeShells['roscore'] = subprocess.Popen('export ROS_IP=$(hostname --all-ip-addresses);roscore;', shell=True)
	try:
		while True:
			refreshShells()
			sleep(3)
	except Exception, e:
		raise e
	finally:
		killAllShells()

if __name__ == '__main__': #Run main.
	main()

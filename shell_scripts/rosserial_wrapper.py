#!/usr/bin/env python
#import rospy
#from std_msgs.msg import Int16

import subprocess
import os.path
import time.sleep
activeShells = {};

def getIP():
	return subprocess.check_output(['hostname','--all-ip-addresses']).strip()

def run_rosserial(port = '/dev/ttyACM0'):
	if (not os.path.exists(port)):
		return None;

	command = ['rosrun', 'rosserial_python', 'serial_node.py', str(port)]

	p = subprocess.Popen(command, l=True)
	return p

def refreshShells():
	for k in activeShells:
		if activeShells[k].poll() is not None:
			# The shell has finished
			activeShells[k].kill()
			del activeShells[k]
			
	for i in range(8):
		path = '/dev/ttyACM{}'.format(i)

		if path in activeShells:
			continue
		else:
			p = run_rosserial(path)
			if p:
				activeShells[path] = p

def killAllShells():
	for k in activeShells:
		activeShells[k].terminate()
		del activeShells[k]
	time.sleep(0.5)

def main():
	try:
		while True:
			refreshShells()
			time.sleep(5)
	except Exception, e:
		raise e
	finally:
		killAllShells()

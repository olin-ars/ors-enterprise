import serial
import subprocess


def find_rate(ser):
	baudrates = [19200, 4800]
	for rate in baudrates:
		ser.baudrate = rate
		ser.open()
		ser.flush()
		print 'here'
		try:
			msg = ser.read(40)
		except:
			msg = ''
		print msg
		if len(msg) > 0:
			chars = msg.split(',')
			if len(chars)>3:
				#Given 40 bytes and longest message size, should still be
				#at least 3 commas
				print 'banana'
				print ser.readline()
				return rate
		ser.close()

def find_device(port):
	while True:
		ser = serial.Serial()
		ser.port = port
		baud = find_rate(ser)
		if baud == 4800:
			return 'airmar'
		elif baud == 19200:
			return 'hemisphere'

def find_all_devices():
	ports = subprocess.check_output("ls /dev/ttyU*", shell=True).strip().split("\n")
	portNames = {}
	for port in ports:
		print port
		portNames[find_device(port)] = port
	return portNames

if __name__ == '__main__':
	

	print find_all_devices()
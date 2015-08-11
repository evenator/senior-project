"""
A module that prints navdata. This was for testing and is deprecated. It doesn't
depend on anything else though, so it's a good way to make sure the hardware's
working.
@author Ed Venator (esv@case.edu)

"""

import socket
import struct
(NAVDATA_DEMO_TAG,
NAVDATA_TIME_TAG,
NAVDATA_RAW_MEASURES_TAG,
NAVDATA_PHYS_MEASURES_TAG,
NAVDATA_GYROS_OFFSETS_TAG,
NAVDATA_EULER_ANGLES_TAG,
NAVDATA_REFERENCES_TAG,
NAVDATA_TRIMS_TAG,
NAVDATA_RC_REFERENCES_TAG,
NAVDATA_PWM_TAG,
NAVDATA_ALTITUDE_TAG,
NAVDATA_VISION_RAW_TAG,
NAVDATA_VISION_OF_TAG,
NAVDATA_VISION_TAG,
NAVDATA_VISION_PERF_TAG,
NAVDATA_TRACKERS_SEND_TAG,
NAVDATA_VISION_DETECT_TAG,
NAVDATA_WATCHDOG_TAG,
NAVDATA_ADC_DATA_FRAME_TAG,
NAVDATA_VIDEO_STREM_TAG) = range(20)
NAVDATA_CKS_TAG = 0xFFFF

def unpack(raw, index):
	""" A function to unpack a navdata tag in a string at a character index.
	@param raw -- A string containing raw navdata
	@param index -- An integer index of the first character of the navdata tag
	"""
	(tag, size) = struct.unpack('<hh', raw[index:(index+4)])
	if(size is 0):
		print "Invalid navdata tag (size=0)"
	elif(tag is NAVDATA_DEMO_TAG):
		(	ctrl_state,
			battery,
			theta,
			phi,
			psi,
			z,
			vx,
			vy,
			vz) = struct.unpack('<IIfffifff',raw[index+4:index+40])
	elif(tag is NAVDATA_CKS_TAG):
		self.checksum(raw[0:index+4], struct.unpack('<I',raw[index+4:index+8]))
		return -1 
	else:
		return -1 #Tag is invalid. Return -1.
	return index+size

port = 5554 #port to listen on
ip = '192.168.1.1'
sock=socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
sock.bind(('', port))
sock.sendto(struct.pack('B',1),(ip, port))
sock.settimeout(1)
count = 0
while True:
	count = (count +1)%1000
	(	ctrl_state,
			battery,
			theta,
			phi,
			psi,
			z,
			vx,
			vy,
			vz) =0,0,0,0,0,0,0,0,0
	try:
		data = sock.recv(1024)
		state = struct.unpack('<I', data[4:8])[0]
		seq = struct.unpack('<I', data[8:12])[0]
		buf = "{0},\t{1},\t{2},\t{3},\t{4},\t{5},\t{6},\t{7},\t{8}".format(	ctrl_state, battery, theta, phi, psi, z, vx, vy, vz)
		print "{3}\t\tECACAVNAPCUWAPTHLGCMBNTTTCUACVVF\n{0}\t\t{1:32b} \t{2}\t{4}".format(seq,state,len(data),count,buf)
		index = 16
		while index > 0:
			index = unpack(data, index)
	except socket.error:
		sock.sendto(struct.pack('B',1),(ip, port)) #Reopen the port by sending it a byte
sock.close()

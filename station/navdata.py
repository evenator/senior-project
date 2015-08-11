"""
This module defines a class Navdata that parses and stores values from a
navdata update from the drone. This module also defines a class 
NavdataListener, which is meant to be run as a thread to listen for navdata on
the appropriate port and marshall it into a threadsafe structure.
Note about units:
	time -- sec
	linear distance -- mm
	linear velocity -- mm/sec
	angle -- radians
	angular velocity -- radians/sec
"""

import struct
import socket
import time
from threading import Thread, Lock
import Queue
import copy
import math
import numpy
import cv
from pso_kalman import PsoKalman

#Enum of possible tags
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

#State masks
FLYING = 1 << 0
VIDEO  = 1 << 1
VISION  = 1 << 2
CONTROL = 1 << 3
ALTITUDE = 1 << 4
USER_FEEDBACK_START = 1 << 5
COMMAND = 1 << 6
TRIM_COMMAND = 1 << 7
TRIM_RUNNING = 1 << 8
TRIM_RESULT = 1 << 9
NAVDATA_DEMO = 1 << 10
NAVDATA_BOOTSTRAP = 1 << 11
MOTORS_BRUSHED  = 1 << 12
COM_LOST = 1 << 13
GYROS_ZERO = 1 << 14
VBAT_LOW = 1 << 15
VBAT_HIGH = 1 << 16
TIMER_ELAPSED = 1 << 17
NOT_ENOUGH_POWER  = 1 << 18
ANGLES_OUT_OF_RANGE = 1 << 19
WIND = 1 << 20
ULTRASOUND = 1 << 21
CUTOUT = 1 << 22
PIC_VERSION = 1 << 23
ATCODEC_THREAD_ON = 1 << 24
NAVDATA_THREAD_ON = 1 << 25
VIDEO_THREAD_ON = 1 << 26
ACQ_THREAD_ON = 1 << 27
CTRL_WATCHDOG = 1 << 28
ADC_WATCHDOG = 1 << 29
COM_WATCHDOG = 1 << 30
EMERGENCY = 1 << 31

class Navdata():
	""" This is an object prototype for a navdata object, which stores all the 
	information from a navdata update.
	
	Fun facts about units:
	Theta: pitch in millidegrees
	Phi: roll in millidegrees
	Psi: yaw in millidegrees
	Z: altitude in centimeters
	Velocities are still unknown
	"""
	state = NAVDATA_BOOTSTRAP #Assume started in bootstrap mode
	header, seq, vision = 0,0,0
	vx,vy,vz,z,theta, phi,psi = 0,0,0,0,0,0,0,
	battery = 0
	
	def __init__(self, raw=None):
		"""Constructor takes in a raw navdata string and unpacks it to the 
			properties.
		@param raw -- A String containing the raw navdata
		"""
		if(raw is not None):
			self.header = struct.unpack('<I', raw[0:4])[0]
			self.state = struct.unpack('<I', raw[4:8])[0]
			self.seq = struct.unpack('<I', raw[8:12])[0]
			self.vision = struct.unpack('<I', raw[12:16])[0]
			#print "{0}\tNavdata state {1:b}  {2}".format(time.clock(),self.state, self.seq)
			index = 16
			while index > 0:
				index = self.__unpack(raw, index)
	
	def check_state(self, mask):
		"""Check the state to see if a flag is set.
		@param mask -- One of the masks defined above
		"""
		#print str.format('Checking State: {0:b} against mask: {1:b} with result: {2:b}',self.state, mask, (self.state & mask))
		if(self.state & mask):
			return True
		return False
	
	
	def __unpack(self, raw, index):
		""" A function to unpack a navdata tag in a string at a character index.
		@param raw -- A string containing raw navdata
		@param index -- An integer index of the first character of the navdata tag
		"""
		(tag, size) = struct.unpack('<hh', raw[index:(index+4)])
		if(size is 0):
			print "Invalid navdata tag (size=0)"
		elif(tag is NAVDATA_DEMO_TAG):
			(	self.ctrl_state,
				self.battery,
				self.theta,
				self.phi,
				self.psi,
				self.z,
				self.vx,
				self.vy,
				self.vz) = struct.unpack('<IIfffifff',raw[index+4:index+40])
			#Unit conversion
			self.theta = math.radians(self.theta/1000)
			self.phi = math.radians(self.phi/1000)
			self.psi = math.radians(self.psi/1000)
			
			#print "{0},\t{1},\t{2},\t{3},\t{4},\t{5},\t{6},\t{7},\t{8}".format(	self.ctrl_state, self.battery, self.theta, self.phi, self.psi, self.z, self.vx, self.vy, self.vz)
		elif(tag is NAVDATA_TIME_TAG):
			self.time = struct.unpack('<f',raw[index:index+4])[0]
		elif(tag is NAVDATA_RAW_MEASURES_TAG):
			self.raw_measures = struct.unpack('<HHHIHHHHHHH',raw[index+4:index+28])
		elif(tag is NAVDATA_PHYS_MEASURES_TAG):
			self.phys_measures = struct.unpack('<fHffIII',raw[index+4:index+30])
		elif(tag is NAVDATA_GYROS_OFFSETS_TAG):
			self.gyro_offsets = struct.unpack('<'+'f'*3,raw[index+4:index+size])
		elif(tag is NAVDATA_EULER_ANGLES_TAG):
			self.euler_angles = struct.unpack('<ff',raw[index+4:index+12])
		elif(tag is NAVDATA_REFERENCES_TAG):
			self.references = struct.unpack('<IIIIIIII',raw[index+4:index+36])
		elif(tag is NAVDATA_TRIMS_TAG):
			self.trims = struct.unpack('<fff',raw[index+4:index+16])
		elif(tag is NAVDATA_RC_REFERENCES_TAG):
			self.rc_references = struct.unpack('<iiiii',raw[index+4:index+24])
		elif(tag is NAVDATA_PWM_TAG):
			pass
		elif(tag is NAVDATA_ALTITUDE_TAG):
			pass
		elif(tag is NAVDATA_VISION_RAW_TAG):
			pass
		elif(tag is NAVDATA_VISION_OF_TAG):
			pass
		elif(tag is NAVDATA_VISION_TAG):
			pass
		elif(tag is NAVDATA_VISION_PERF_TAG):
			pass
		elif(tag is NAVDATA_TRACKERS_SEND_TAG):
			pass
		elif(tag is NAVDATA_VISION_DETECT_TAG):
			pass
		elif(tag is NAVDATA_WATCHDOG_TAG):
			self.watchdog = struct.unpack('<i',raw[index:index+4])
		elif(tag is NAVDATA_ADC_DATA_FRAME_TAG):
			pass
		elif(tag is NAVDATA_VIDEO_STREM_TAG):
			pass
		elif(tag is NAVDATA_CKS_TAG):
			self.checksum(raw[0:index+4], struct.unpack('<I',raw[index+4:index+8]))
			return -1 
		else:
			return -1 #Tag is invalid. Return -1.
		return index + size
		
class NavdataListener(Thread):
	"""A thread class that listens for navdata"""
	q=Queue.Queue()
	port = 5554 #port to listen on
	__navdata = Navdata()
	__navdata_lock = Lock()
	__state_lock = Lock()
	__command_lock = Lock()
	sequence = 0
	
	#Create Kalman filter, state, and command vectors
	kalman = PsoKalman()
	u = cv.CreateMat(4, 1, cv.CV_32FC1)
	z = cv.CreateMat(5, 1, cv.CV_32FC1)
	sys_state = cv.CreateMat(8, 1, cv.CV_32FC1)
	sys_time = time.time()
	
	def __init__(self, drone): #ip is drone's IP
		"""Constructor takes in a drone object (see drone.py) and sets the IP
		address. The drone object is used for communication with the command
		thread.
		@param drone -- A Drone object.
		"""
		print "Initializing Navdata Listener..."
		self.drone = drone
		self.ip = drone.ip #drone's IP
		Thread.__init__(self)
	
	#Set up navdata stream
	def __setup(self):
		"""Starts the navdata listener. Creates the UDP socket and bind it, then
		sends a single byte to the drone to start the navdata stream."""
		print "Starting Navdata Listener..."
		#Create UDP socket
		self.__socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		#Bind navdata port
		self.__socket.bind(('', self.port))
		#Send a byte over the navdata port just so it knows the station is alive
		self.__socket.sendto(struct.pack('B',1),(self.ip, 5554))
		self.__socket.settimeout(.1)
		
	def run(self):
		"""The main method of the thread, as called by start(). Runs __setup(),
		then continuously listens for navdata forever. Stores navdata as 
		Navdata objects in self.__navdata (protected by a lock to avoid race
		conditions.)
		Performs all watchdog feeding (by communicating with the drone's
		command thread.)
		"""
		self.__setup()
		self.kill = False
		self.sequence = 0
		while not self.kill:
			#Receive Data
			try:
				data=self.__socket.recv(4096)
				#state = struct.unpack('<I', data[4:8])[0]
				#seq = struct.unpack('<I', data[8:12])[0]
				#print "\t\tECACAVNAPCUWAPTHLGCMBNTTTCUACVVF\n{0}\t\t{1:b} \t{2}".format(seq,state,len(data))
				navdata = Navdata(data)
				dt = time.time()-self.sys_time
				print "dt:\t",dt
				self.sys_time = time.time()
				self.z[0, 0], self.z[1, 0], self.z[2, 0], self.z[3, 0], self.z[4, 0] = navdata.vx, navdata.vy, navdata.z, navdata.vz, navdata.psi
				with self.__state_lock:
					self.sys_state = self.kalman.update(self.z, self.u, dt)
				if navdata.check_state(COM_WATCHDOG):
					if not self.__navdata.check_state(COM_WATCHDOG):
						print "WATCHDOG"
					self.sequence = 0
					if not navdata.check_state(NAVDATA_BOOTSTRAP):
						#Reset watchdog
						self.drone.reset_watchdog()
				else:
					if self.__navdata.check_state(COM_WATCHDOG):
						print "WATCHDOG Cleared"
				if navdata.seq > self.sequence:
					sequence = navdata.seq
					with self.__navdata_lock:
						self.__navdata=navdata						
			except socket.error:
				self.__socket.sendto(struct.pack('B',1),(self.ip, 5554))
			time.sleep(.05)
		print "Shutting down Navdata Listener..."
		self.__socket.close()
		
	def get_navdata(self):
		"""Threadsafe way to get the current navdata.
		@return a Navdata object representing the drone's current state.
		"""
		with self.__navdata_lock:
			if self.__navdata is None:
				return None
			return copy.deepcopy(self.__navdata)
	def set_command(self, u):
		"""Threadsafe way to set the current command.
		"""
		with self.__command_lock:
			self.u = u
	def get_state(self):
		"""Threadsafe way to get the current state vector.
		@return a cvMatrix representing the drone's current state."""
		with self.__state_lock:
			return self.sys_state

"""
A module for UNIX socket communication with the PSO/control loop for the Drone 
project.
Classes:
UISender -- should be run from the UI process to communicate with the PSO process
UIListener -- run from within the PSO to communicate with a UISender in the
	UI process
PlannerSender -- run from within the planner/mapper process to communicate with
	the PSO process

Other Objects:
enum(EMERGENCY, SHUTDOWN, FLYING, OVERRIDE, COMMAND, TRIM) -- describes the state
	of the UI.
"""

import cv
import numpy
import os
import socket
import struct
from threading import Thread, Lock
import Queue
import time
import debuglogger
import navdata

(EMERGENCY, SHUTDOWN, FLYING, OVERRIDE, COMMAND, TRIM, RESET) = range(7)

# Keys into the UI dictionary
(	X,
	Y,
	Z,
	VX,
	VY,
	VZ,
	YAW,
	VYAW,
	EMERGENCYFLAGS,
	FLYINGSTATE,
	BAT,
	FLOOR_RANGE,
	CEILING_RANGE
) = range(13)

class UISender():
	""" A class to communicate with the pso from the UI
		This class should be imported into the UI process and used to send commands
		to the control loop and receive data from the PSO.
	"""
	
	manual_override=True
	__connected=False
	data = {'x':0, 'vx':0, 'y':0, 'vy':0, 'z':0, 'vz':0, 'yaw':0, 'vyaw':0, 'floor_range':0, 'ceiling_range':0, 'emergency':False, 'flying':False, 'bat':0}
	
	def __init__(self):
		"""The initializer takes no arguments. It creates a UNIX stream socket
		and runs self.connect(). If connect throws a socket error, it prints an
		error message and keeps trying once a second until it connects.
		"""
		self.__s= socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		while True:
			try:
				self.connect()
				break
			except socket.error,  msg:
				print "UI Sender connection failed with ",msg
			time.sleep(1)
		if (not self.__connected):
			print "UI Sender cannot connect"
		return None
	
	def connect(self):
		"""This function is used to connect to the PSO process. Attempts to connect
		to the UNIX socket at /tmp/ui_pso_socket. Prints "UI Sender connected" 
		and returns True on success. Throws a socket.error on failure."""
		self.__s.connect("/tmp/ui_pso_socket")
		self.__s.settimeout(.02)
		print "UI Sender connected\n"
		self.__connected=True
		return True
	
	def set_mode(self, manual_override): 
		""" Set the manual override mode. Returns True.
		manual_override -- True for gamepad control, False for waypoint or autonomous
		"""
		self.manual_override=bool(manual_override)
		message=struct.pack('c?dddd','u',self.manual_override,0.,0.,0.,0.)
		self.__s.send(message)
		return True
		
	def update(self, pitch, roll, yaw_rate, vertical_rate):
		""" Send a joystick command. (Ignored by the control loop unless manual 
		override is on.)
		pitch -- Floating point -1 to 1. Controls y speed.
		roll -- Floating point -1 to 1. Controls x speed.
		yaw_rate -- Floating point -1 to 1. Controls theta rotation rate.
		vertical_rate -- Floating point -1 to 1. Controls z speed."""
		message=struct.pack('c?dddd','u',self.manual_override,pitch,roll,yaw_rate,vertical_rate)
		self.__s.send(message)
		return True
	
	def get_state(self):
		""" Get the state of the drone. (Note: this function unfinished! Suggest changes!)
		Returns a Python dict object containing the following entries:
		x		-- left-right position of drone in world space (mm)
		y		-- forward-back position of drone in world space (mm)
		z		-- height of drone in world space (mm)
		yaw	-- angle of drone in world space. 0 is along the +x axis. +90 is along the +y axis. (deg)
		vx		-- derivative of x
		vy		-- derivative of y
		vz		-- derivative of z
		vyaw	-- derivative of theta
		
		floor_range 	-- distance to floor, according to ultrasonic rangefinder (mm)
		ceiling_range	-- distance to ceiling, according to ultrasonic rangefinder (mm)
		
		emergency	-- whether the drone is in emergency mode
		flying		-- whether the drone is flying
		bat		-- the battery percentage
		"""
		
		message=struct.pack('c', 'r')
		self.__s.send(message)
		#ack = self.__s.recv(8)
		#print "Receiving state data:"
		for i in range(5):
			try:
				data = self.__s.recv(4096)
				#print "\t",data
				if len(data) and data[0] == 'r':
					data = struct.unpack_from('cfffffffffffff??f',data)
					data=data[1:]
					#print data
					data = dict(zip(('x', 'vx', 'y', 'vy', 'z', 'vz', 'yaw', 'vyaw', 'floor_range', 'pitch', 'roll', 'yaw', 'ceiling_range', 'emergency', 'flying', 'bat'), data))
					#print data
					#print "\tReturning state data"
					self.data = data
					return data
			except socket.timeout:
				pass
		return self.data
	
	def land_drone(self):
		"""Command the control loop to land the drone. Returns True"""
		message=struct.pack('c?','l',False)
		self.__s.send(message)
		return True

	def liftoff_drone(self):
		"""Command the control loop to lift off the drone. Waits ten seconds
		for the drone to take off before returning. Returns True."""
		message=struct.pack('c?','l',True)
		self.__s.send(message)
		print "Liftoff sent"
		#time.sleep(10);
		return True
		
	def trim_drone(self):
		"""Command the drone to reset trim values. ONLY SEND THIS WHEN THE DRONE
		IS ON A FLAT SURFACE. It's probably a good idea to have a button for this,
		or to send it immediately before takeoff."""
		message=struct.pack('c','t')
		self.__s.send(message)
		return True
	
	def reset_pso(self):
		"""Command the pso to reset x, y, z, and theta to 0. Probably not a good
		idea to send this while flying."""
		message=struct.pack('c','c');
		self.__s.send(message)
		return True
	
	def emergency_stop(self):
		"""Order an emergency stop. This kills the motors and the drone falls.
		As of right now, the only way to reset the emergency state is with the 
		reset button on the drone."""
		message=struct.pack('c?','e',True)
		ack=''
		#while not len(ack) or ack[0] != 'e':
		self.__s.send(message)
		#	ack = self.__s.recv(8)
		return True
		
	def shutdown(self):
		"""Land the drone and shut down the PSO/control loop process."""
		message=struct.pack('c','s')
		ack = ''
		#while not len(ack) or ack[0] != 's':
		self.__s.send(message)
		#	ack = self.__s.recv(8)
		self.__s.close()
		return True
		
	def send(self, message):
		while True:
			try:
				return self.__s.send(message)
			except socket.error:
				print "UI Sender has lost connection."
				while True:
					try:
						self.connect()
						break
					except socket.error,  msg:
						print "UI Sender connection failed with ",msg
					time.sleep(1)

class UIListener(Thread):
	"""UIListener is a thread class to listen for messages from the UI.
		It is set up and run in the main() function of the PSO. It receives
		messages sent over UNIX socket from a UISender in another process and
		updates the ui_state property accordingly."""

	__state = (0, 0, 0, 0, 0, 0, 0, 0)
	__nav = navdata.Navdata()
	__ui_lock = Lock()
	__state_lock = Lock()
	__ui_state = [False, False, False, True, [0, 0, 0, 0], False, False] #(emergency, shutdown, flying, override, command, trim, reset_pso)
	
	def __init__(self):
		"""Sets up a socket and binds it to /tmp/ui_pso_socket"""
		print "Initializing UI Listener..."
		Thread.__init__(self)
		try:
			os.remove("/tmp/ui_pso_socket")
		except OSError:
			pass
		self.__s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		self.__s.bind("/tmp/ui_pso_socket")
	
	def __read(self):
		"""Tries to receive data from the connection. On  socket error (meaning
		the connection has been lost, prints "Waiting to accept UI connection"
		and attempts to establish a new connection. On receipt of data,
		parses the message and updates the ui_state (ui_state is accessed using
		ui_lock to prevent race conditions). If an unrecognized command is 
		recieved, the command is interpreted as an emergency command"""
		try:
			data = self.__conn.recv(64)
		except socket.error:
			print "Waiting to accept UI connection\n"
			self.__conn, addr = self.__s.accept()
			print "UI Connected\n"
			return None
		if data:
			data_type=struct.unpack_from('c',data)[0]
			#self.__conn.send(data_type) #ack
			if data_type=='s':
				print ('SHUTDOWN')
				with self.__ui_lock:
					self.ui_state[FLYING] = False #Land the drone
					self.ui_state[SHUTDOWN] = True #Shut down the drone
			elif data_type=='t':
				with self.__ui_lock:
					self.__ui_state[TRIM] = True
			elif data_type=='l':
				with self.__ui_lock:
					self.__ui_state[FLYING] = struct.unpack_from('c?',data)[1]
			elif data_type=='u':
				command = struct.unpack_from('c?dddd',data)
				#print command, " command"
				with self.__ui_lock:
					self.__ui_state[OVERRIDE] = command[1] #Set override mode
					self.__ui_state[COMMAND] = command[2:] #Set command
					#print self.__ui_state, "self.__ui_state"
			elif data_type=='c':
				with self.__ui_lock:
					self.__ui_state[RESET] = True
					self.__ui_state[EMERGENCY] = False
				pass
			elif data_type=="\00": #empty message. Ignore
				return None
			elif data_type=='r': #request for state and navdata
				with self.__state_lock:
					state = self.__state
					nav = self.__nav
				data = struct.pack("cfffffffffffff??f",
					'r',
					state[0], 
					state[1], 
					state[2], 
					state[3], 
					state[4], 
					state[5], 
					state[6], 
					state[7],
					nav.z,
					nav.theta,
					nav.phi,
					nav.psi, 
					0, 
					nav.check_state(navdata.EMERGENCY), 
					nav.check_state(navdata.FLYING), 
					nav.battery
				)
					#print data
				self.__conn.send(data)
			elif data_type=='e': #Emergency command.
				with self.__ui_lock:
					self.__ui_state[EMERGENCY] = True
	
	def run(self):
		"""Main method of the thread. Starts listening on the socket and waits
		for a connection. Once connected, runs self.__read() in a loop until
		self.__run is set to false (in stop()). Then prints a message that it is
		stopping and closes the connection and socket."""
		print "Starting UI socket listener..."
		self.__s.listen(5)
		print "Waiting to accept UI connection\n"
		self.__conn, addr = self.__s.accept()
		print "UI Connected\n"
		self.__run = True
		while (self.__run and self.__conn):
			self.__read()
		print "Stopping UI socket listener..."		
		# Close the connection
		self.__conn.close()
		self.__s.close()
	
	def stop(self):
		"""Sets self.__run to False (to stop the loop in __run()) and then returns
		True"""
		self.__run=False
		return True
		
	def get_ui(self):
		"""Returns the __ui_state (using a lock to prevent race conditions)""" 
		with self.__ui_lock:
			return self.__ui_state
	def clear_flag(self, flag):
		"""Clears the field in the ui_state represented by flag (using a lock to 
		prevent race conditions). Returns False on an improper flag and True on
		success.
		
		flag -- one of the values in enum(EMERGENCY, SHUTDOWN, FLYING, OVERRIDE, COMMAND, TRIM)
			except for COMMAND"""
		if flag not in (EMERGENCY, SHUTDOWN, FLYING, OVERRIDE, TRIM, RESET):
			return False
		with self.__ui_lock:
			self.__ui_state[flag] = False
		return True
		
	def set_state(self, state, nav):
		"""Sets the internal state from navdata and other sources, which is delivered as a 
		dictionary when the state is requested"""
		with self.__state_lock:
			self.__state = (state[0, 0], state[1, 0], state[2, 0], state[3, 0], state[4, 0], state[5, 0], state[6, 0], state[7, 0])
			self.__nav = nav

class PlannerSender():
	""" A class to communicate with the pso from the planner/mapper
		This class should be imported into the planner process and used to send 
		commands to the control loop and receive data from the PSO.
	"""
	
	__connected=False
	
	def __init__(self):
		"""The initializer takes no arguments. It creates a UNIX stream socket
		and runs self.connect(). If connect throws a socket error, it prints an
		error message and keeps trying once a second until it connects.
		"""
		self.__s= socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		while True:
			try:
				self.connect()
				break
			except socket.error,  msg:
				print "Planner Sender connection failed with ",msg
			time.sleep(1)
		if (not self.__connected):
			print "Planner Sender cannot connect"
		return None
	
	def connect(self):
		"""This function is used to connect to the PSO process. Attempts to connect
		to the UNIX socket at /tmp/planner_pso_socket. Prints "Planner Sender connected" 
		and returns True on success. Throws a socket.error on failure."""
		self.__s.connect("/tmp/planner_pso_socket")
		print "Planner Sender connected\n"
		self.__connected=True
		return True
	
	def delta(self, x, y, z, yaw):
		"""Send a message to the PSO that the SLAM algorithm has a delta.
		If the SLAM algorithm shows the drone's position is different from
		that predicted by the PSO, send the difference to the PSO for correction.
		x -- The delta in the x direction (mm)
		y -- The delta in the y direction (mm)
		z -- The delta in the z direction (mm)
		yaw -- The delta in the yaw direction (deg)
		"""
		message=struct.pack('cdddd', 'd', x, y, z, yaw)
		self.__s.send(message)
		return True
		
	def add_waypoint(self, x, y, z, yaw):
		"""Send a waypoint to the control loop.
		Adds a waypoint to the control loop's waypoint queue. The drone will
		proceed through the queues in the way point in a straight line from
		point to point.
		x -- The x position of the waypoint in world space (mm)
		y -- The y position of the waypoint in world space (mm)
		z -- The z position of the waypoint in the world space (mm)
		yaw -- The yaw angle of the waypoint in the world space (rad)
		"""
		message=struct.pack('cdddd', 'a', x, y, z, yaw)
		self.__s.send(message)
		print "Sent ",message, " --> ",struct.unpack_from('cdddd', message)
		return True
	
	def clear_waypoints(self):
		"""Clear the waypoint queue in the control loop.
		Empties the queue of waypoints in the control loop. The drone will stop
		until it recieves more waypoints (or a manual override).
		"""
		message=struct.pack('c', 'c')
		self.__s.send(message)
		return True
		
	def get_state(self):
		""" Get the state of the drone.
		Returns a cvMat containing the state:
		x		-- left-right position of drone in world space (mm)
		y		-- forward-back position of drone in world space (mm)
		z		-- height of drone in world space (mm)
		yaw		-- angle of drone in world space. 0 is along the +x axis. +90 is along the +y axis. (deg)
		vx		-- derivative of x
		vy		-- derivative of y
		vz		-- derivative of z
		vyaw	-- derivative of theta
		"""
		
		message=struct.pack('c', 'r')
		self.__s.send(message)
		ack = self.__s.recv(8)
		data = self.__s.recv(4096)
		if data:
			data = struct.unpack_from('ffffffff',data)
			return cv.fromarray(numpy.array(zip(data)))
		else:
			print "no state received"
		return None
		
class PlannerListener(Thread):
	"""PlannerListener is a thread class to listen for messages from the Planner.
		It is set up and run in the main() function of the PSO. It receives
		messages sent over UNIX socket from a PlannerSender in another process and
		updates the waypoint queue and SLAM delta property accordingly."""

	__state = cv.CreateMat(8, 1, cv.CV_32FC1)

	__planner_lock = Lock()
	__state_lock = Lock()
	__delta = cv.CreateMat(4, 1, cv.CV_32FC1)
	waypoints = Queue.Queue()
	
	def __init__(self):
		"""Sets up a socket and binds it to /tmp/planner_pso_socket"""
		print "Initializing Planner Listener..."
		Thread.__init__(self)
		try:
			os.remove("/tmp/planner_pso_socket")
		except OSError:
			pass
		self.__s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		self.__s.bind("/tmp/planner_pso_socket")
	
	def __read(self):
		"""Tries to receive data from the connection. On  socket error (meaning
		the connection has been lost, prints "Waiting to accept Planner connection"
		and attempts to establish a new connection. On receipt of data,

		parses the message and alters the queue or sets the delta as appropriate"""
		try:
			data = self.__conn.recv(64)
		except socket.error:
			print "Waiting to accept Planner connection\n"
			self.__conn, addr = self.__s.accept()
			print "Planner Connected\n"
			return None
		if data:
			data_type=struct.unpack_from('c',data)[0]
			self.__conn.send(data_type) #ack
			if data_type=='c':
				try:
					while True:
						self.__waypoints.get_nowait()
				except Queue.Empty:
					pass
			elif data_type == 'a':
				temp = struct.unpack_from('cdddd',data)
				waypoint = cv.CreateMat(4, 1, cv.CV_32FC1)
				waypoint[0, 0] = temp[1]
				waypoint[1, 0] = temp[2]
				waypoint[2, 0] = temp[3]
				waypoint[3, 0] = temp[4]
				self.waypoints.put(waypoint)
			elif data_type == 'd':
				with self.__planner_lock:
					temp = data.unpack_from('dddd',data[1:])
					self.__delta[0, 0] = temp[0][0]
					self.__delta[1, 0] = temp[0][1]
					self.__delta[2, 0] = temp[0][2]
					self.__delta[3, 0] = temp[0][3]
			elif data_type == 'r': #Send State data
				with self.__state_lock:
					data = struct.pack("ffffffff", 
						self.__state[0,0], 
						self.__state[1,0], 
						self.__state[2,0], 
						self.__state[3,0], 
						self.__state[4,0], 
						self.__state[5,0], 
						self.__state[6,0], 
						self.__state[7,0])
					self.__conn.send(data)
			else: #Unrecognized command.
				return None
	
	def run(self):
		"""Main method of the thread. Starts listening on the socket and waits
		for a connection. Once connected, runs self.__read() in a loop until

		self.__run is set to false (in stop()). Then prints a message that it is
		stopping and closes the connection and socket."""
		print "Starting Planner socket listener..."
		self.__s.listen(5)
		print "Waiting to accept Planner connection\n"
		self.__conn, addr = self.__s.accept()
		print "Planner Connected\n"
		self.__run = True
		while (self.__run and self.__conn):
			self.__read()
		print "Stopping UI socket listener..."		
		# Close the connection
		self.__conn.close()
		self.__s.close()
	
	def stop(self):
		"""Sets self.__run to False (to stop the loop in __run()) and then returns
		True"""
		self.__run=False
		return True
		
	def get_delta(self):
		"""Returns the delta (using a lock to prevent race conditions)""" 
		with self.__planner_lock:
			return self.__delta
		
	def set_state(self, state):
		"""Sets the internal state"""
		with self.__state_lock:
			self.__state = state
	

"""
This module defines a class Drone that represents a drone. Methods in this 
class are used to set drone properties and commands and to get data from the
drone."""

import socket
import time
import struct
import Queue
import navdata
from navdata import Navdata, NavdataListener
import command
from command import Command, CommandSender
import math
import cv

class Drone:
	"""A class for interacting with a Parrot AR.Drone.
	Properties:
	 - ip -- the ip address of the drone
	"""
	
	ip = "192.168.1.1"
	__u = cv.CreateMat(4, 1, cv.CV_32FC1)
	
	def __init__(self, ip):
		"""Initializes the drone object and sets properties.
		Sets self.ip to the ip parameter. Creates a CommandSender thread
		as a daemon (daemons are automatically killed when the main process
		ends). Creates a NavdataListener thread as a daemon. Starts both of these
		threads. Puts an FTRIM command on the command queue. (This might not
		do anything because the drone may get it before it boots fully.)
		ip -- the ip address of the drone.
		"""
		print("Initializing drone object...")
		self.ip = ip
		#Start Command sender and Navdata listener threads
		self.__cmd = CommandSender(self)
		self.__cmd.daemon = True
		self.__nav = NavdataListener(self)
		self.__nav.daemon = True
		self.__cmd.start()
		self.__nav.start()
		
		#Reset Trims
		self.__cmd.q.put(Command(command.FTRIM))
		
	
	def reset_watchdog(self):
		"""Puts a RESET_WATCHDOG command on the CommandSender thread's queue."""
		self.__cmd.q.put(Command(command.RESET_WATCHDOG))
		
	def get_navdata(self, current=True):
		"""Gets the drone's navdata object.
		current -- If set to False, pulls the navdata off the queue. Otherwise
			returns the most recent navdata. Defaults to True.
		"""
		if not current:
			return self.__nav.q.get()
		return self.__nav.get_navdata()
		
	def get_state(self):
		"""Gets the drone's state vector as a cv matrix"""
		return self.__nav.get_state()
	
	def takeoff(self):
		"""Puts a TAKEOFF command on the CommandSender thread's queue."""
		self.__cmd.q.put(Command(command.TAKEOFF))
		return True
		
	def land(self):
		"""Puts a LAND command on the CommandSender thread's queue."""
		self.__cmd.q.put(Command(command.LAND))
		return True
		
	def go(self, pitch, roll, yawRate, gaz):
		"""Puts a GO command on the CommandSender thread's queue. This is used
		to order the drone to move.
		pitch -- A float from -1 to 1 commanding the drone's pitch angle
		roll -- A float crom -1 to 1 commanding the drone's roll angle
		yawRate -- A float from -1 to 1 commanding the drone's yaw rotation rate
		gaz -- A float from -1 to 1 commanding the drone's vertical velocity
		"""
		if math.fabs(pitch) > .01 or math.fabs(roll) > .01 or math.fabs(yawRate) > .01 or math.fabs(gaz) > .01:
			int_pitch=struct.unpack('i',struct.pack('f',pitch))[0]
			int_roll=struct.unpack('i',struct.pack('f',roll))[0]
			int_yawRate=struct.unpack('i',struct.pack('f',yawRate))[0]
			int_gaz=struct.unpack('i',struct.pack('f',gaz))[0]
			self.__cmd.q.put(Command(command.GO,(int_roll, int_pitch, int_gaz, int_yawRate)))
			self.__u[0, 0] = pitch
			self.__u[1, 0] = roll
			self.__u[2, 0] = gaz
			self.__u[3, 0] = yawRate
		else:
			self.__cmd.q.put(Command(command.GO,(0,))) #hover
			cv.Zero(self.__u)
		self.__nav.set_command(self.__u)
		return True
		
	def hover(self):
		"""Puts a GO command on the CommandSender thread's queue with all 
		velocities set to 0."""
		self.__cmd.q.put(Command(command.GO))
		return True
		
	def reset(self):
		"""Resets the kalman filter"""
		self.__nav.kalman.reset()
		
	def reset_emergency(self):
		"""Resets the drone if it is in the emergency state by putting an 
		ACK_EMERGENCY and an EMERGENCY command on the CommandSender thread's 
		queue. This function works."""
		self.__cmd.q.put(Command(command.ACK_EMERGENCY))
		self.__cmd.q.put(Command(command.EMERGENCY))
		return True
		
	def trim(self):
		"""Puts an FTRIM command on the CommandSender thread's queue. Should
		only be called when the drone is on a flat surface!"""
		self.__cmd.q.put(Command(command.FTRIM))
		return True
	
	def emergency(self):
		"""Orders the drone into an emergency state by putting an EMERGENCY
		command on the CommandSender thread's queue. Prints the message
		\"Drone is in emergency state.\"
		"""
		self.__cmd.q.put(Command(command.EMERGENCY))
		print "Drone is in emergency state (ordered by user)."
		return True
	
	def kill(self):
		"""Kills the NavdataListener and CommandSender threads, then waits for
		them to die."""
		print "Killing Drone..."
		self.__nav.kill = True
		print "Killing Navdata Loop..."
		self.__nav.join()
		self.__cmd.kill = True
		print "Killing Command Loop..."
		self.__cmd.join()
		

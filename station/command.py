"""
A module with classes to send commands to the drone.
Classes:
Command -- an object representing a command to send to the drone
CommandSender -- a thread object that sends commands to the drone

Other Objects:
An enum representing the possible command types:
	LAND
	TAKEOFF
	GO -- sets pitch, roll, yaw rate, and z rate
	FTRIM -- set angular trims to 0 (drone should be on flat surface)
	EMERGENCY -- Toggle emergency mode
	ACK_EMERGENCY -- Acknowledge emergency state. (Must be sent before leaving
		emergency mode.)
	RESET_WATCHDOG -- Reset the command watchdog on the drone. Must be sent if
		there is no command for >30ms (indicated by a flag in the navdata).
	NAVDATA -- Command the drone to send a navdata stream.
	PMODE -- Undocumented command that parrot sends. (???)
	MISC --  Undocumented command that parrot sends. (???)
	NAVDATA_ACK -- Acknowledge navdata stream from the drone.
"""

import struct
import socket
import time
from threading import Thread, Lock
import Queue
import navdata
import math

(LAND,
TAKEOFF,
GO,
FTRIM,
EMERGENCY,
ACK_EMERGENCY,
RESET_WATCHDOG,
NAVDATA,
PMODE,
MISC,
NAVDATA_ACK,
SET_MAX_ANGLE) = range(12)
	

class Command():
	"""An object representing a command to the drone."""
	
	def __init__(self, cmd_type, params = None):
		"""Initialize the command object properties.
		cmd_type -- one of the command types defined in the enum above
		params -- a tuple containing any parameters that the command takes
			Required for GO, PMODE, and MISC commands.
		"""
		self.cmd_type = cmd_type
		self.params = params
	
	def to_string(self, seq = 1):
		"""Returns the string command in the format to be sent over the network.
		Includes the sequence number in the command string. The sequenc number
		should be incremented for each command sent. The drone ignores 
		commands with a sequence less than the previous command received 
		(unless seq = 1, which resets the counter.)
		seq -- the sequence number of the command
		"""
		if self.cmd_type == LAND:
			return 'AT*REF={0},{1}\r'.format(seq,290717696)
		elif self.cmd_type == TAKEOFF:
			return 'AT*REF={0},{1}\r'.format(seq,290718208)
		elif self.cmd_type == GO:
			if len(self.params) == 4:
				return 'AT*PCMD={0},1,{1},{2},{3},{4}\r'.format(seq,self.params[0], self.params[1], self.params[2], self.params[3])
			else:
				return 'AT*PCMD={0},0,0,0,0,0\r'.format(seq)
		elif self.cmd_type == FTRIM:
			return 'AT*FTRIM={0}\r'.format(seq)
		elif self.cmd_type == EMERGENCY:
			return 'AT*REF={0},256\r'.format(seq)
		elif self.cmd_type == ACK_EMERGENCY:
			return 'AT*REF={0},0\r'.format(seq)
		elif self.cmd_type == RESET_WATCHDOG:
			return 'AT*COMWDG={0}\r'.format(seq)
		elif self.cmd_type == NAVDATA:
			return 'AT*CONFIG={0},"general:navdata_demo","TRUE"\r'.format(seq)
		elif self.cmd_type == PMODE:
			return 'AT*PMODE={0},{1}\r'.format(seq, self.params[0])
		elif self.cmd_type == MISC:
			return 'AT*MISC={0},{1},{2},{3},{4}'.format(seq,self.params[0], self.params[1], self.params[2], self.params[3])
		elif self.cmd_type == NAVDATA_ACK:
			return 'AT*CTRL={0},5,0\r'.format(seq)
		elif self.cmd_type == SET_MAX_ANGLE:
			print self.params
			return 'AT*CONFIG={0},"control:euler_angle_max:","{1:.2f}"'.format(seq, self.params[0])
		else:
			return ''

class CommandSender(Thread):
	"""A thread class to send commands to the drone every 25ms on port 5556.
	Properties:
	 - q -- A queue of commands to be sent to the drone
	 - __drone -- The drone object that this CommandSender is associated with
	 - __port -- The port to send on. Set to 5556
	 - __sequence -- The current sequence number of the command. Starts at 1.
	 - __sock -- The UDP socket pointer to send commands to.
	 - __cmd -- The most recent command pulled off the queue.
	"""
	
	q=Queue.Queue()
	__port = 5556 #port to send on
	__sequence = 1
	__sock=socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
	__cmd = ''
	
	def __init__(self, drone):
		"""Creates a sender that sends commands to the drone represented by the
		Drone object in the drone parameter. Sets the __drone and __ip properties
		from the drone parameter.
		
		drone -- A Drone object to associate with this CommandSender. Each Drone
			should have one and only one CommandSender and vice versa.
		"""
		
		print "Initializing Drone Command Sender..."
		self.__drone = drone
		self.__ip = drone.ip #drone's IP
		Thread.__init__(self)
		
	def run(self):
		"""Main function of the thread. Resets the sequence to 1, then puts an
		FTRIM command on the queue. Then runs a loop every 25 ms until self.kill
		is True AND q is empty.
		On each iteration, checks if the drone is in bootstrap mode and runs
		boot_drone if it is. Then runs __send_cmd() and sleeps until 25 ms have
		passed.
		"""  
		self.kill = False
		self.__seq = 1
		
		self.q.put(Command(FTRIM))
		
		while (not self.kill) or (not self.q.empty()):
			#If the drone is in bootstrap mode, boot it
			if(self.__drone.get_navdata().check_state(navdata.NAVDATA_BOOTSTRAP)):
				print "Attempting to Boot Drone"
				self.boot_drone()

			#Compute next loop iteration deadline
			deadline = time.clock() + .025
		
			#Send Command
			self.__send_cmd()
		
			#Sleep until next deadline
			current = time.clock()
			if current < deadline:
				time.sleep(deadline-current)
		
		print "Drone Command Sender Stoppping...\n"
	
	def boot_drone(self):
		"""Boots the drone (initializes the navdata stream.
		Puts a NAVDATA, PMODE(2), and MISC(2,20,2000,3000) command on q.
		Then checks the navdata once a second for at most 20 seconds. Once the
		COMMAND flag is set in the navdata state, it puts a NAVDATA_ACK command
		on q.
		"""
		self.q.put(Command(NAVDATA))
		self.q.put(Command(PMODE,(2,)))
		self.q.put(Command(MISC,(2,20,2000,3000)))
		
		retry = 20
		bcontinue = True
		next = 0
		
		while bcontinue and retry:
			if next==0:
				if self.__drone.get_navdata().check_state(navdata.COMMAND):
					print "[CONTROL] Processing the current command...\n"
					next += 1
			else:
				self.q.put(Command(NAVDATA_ACK))
				if self.__drone.get_navdata().check_state(navdata.COMMAND):
					print "[CONTROL] Ack control Ok, send navdata demo\n"
					bcontinue = False
			time.sleep(1)
			retry -= 1
		self.q.put(Command(SET_MAX_ANGLE, (.05,)))
	
	def __send_cmd(self):
		"""If there are commands on the queue, sends the first five commands on
		the queue as a single block. If there are no commands on the queue,
		resends the last command sent (to avoid watchdog problems). Advances
		__seq by the number of commands sent.
		"""
		#Get command from the queue, if there is one
		cmd = ''
		flag = False
		start_seq = self.__seq
		while not self.q.empty() and self.__seq - start_seq < 5:
			self.__cmd = self.q.get()
			cmd += self.__cmd.to_string(self.__seq)
			self.__seq += 1
			flag = True
		if not flag:
			cmd = self.__cmd.to_string(self.__seq)
			self.__seq += 1
		
		#Send command
		#print '{0}\t'.format(time.clock()),cmd
		self.__sock.sendto(cmd, (self.__ip, self.__port))
		while start_seq <self.__seq and flag:
			self.q.task_done()
			start_seq += 1

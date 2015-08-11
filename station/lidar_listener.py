import struct
import socket
import string
import threading, Queue
import cv, numpy, time
import math, signal

class LidarListener(threading.Thread):
	port = 5560
	kill = False
	
	def __init__(self, q):
		print "Initializing LIDAR Listener..."
		threading.Thread.__init__(self)
		self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.__socket.bind(('', self.port))	
		self.__q = q
	
	def run(self):
		print "Running the thread..."
		while not self.kill:
			try:
				raw_data =self.__socket.recv(4096, socket.MSG_DONTWAIT)
				if not raw_data:
					print "no raw!"
					self.kill = True
				else:
					data = self.decode(raw_data)
					#print data
					self.__q.put(data)
			except socket.error as ex:
				if (ex[0] != 11):
					print ex
					self.kill = True
			except Exception as ex:
				raise ex
		print "Shutting down LIDAR listener..."
		self.__socket.close()		
		
	def decode(self, raw):
		lines = string.split(raw, "\n")
		data = [];
		for line in lines:
			for index in range(0,len(line)-1,2):
				value = struct.unpack('B',line[index:index+1])[0] - 0x30
				value <<= 6
				value &= ~0x003f
				value |= struct.unpack('B',line[index+1:index+2])[0] - 0x30
				data.append(value)
		return data


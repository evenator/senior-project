#!/usr/bin/env python
import struct
import socket
import string
import threading, Queue
import cv, numpy, time
import math, signal
import roslib; roslib.load_manifest('drone_planner')
import rospy
from sensor_msgs.msg import LaserScan


def lidar_listener():
	port = 5560
	kill = False
	
	#init
	rospy.loginfo("Initializing LIDAR Listener...");
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind(('', port))
	laserpub = rospy.Publisher('/scan', LaserScan)
	rospy.init_node('lidar_listener')
	
	message = LaserScan();
	message.angle_max = 4.1887
	message.angle_increment = .0062832
	message.scan_time = .1
	message.range_min = .02
	message.range_max = 4.0
	message.header.frame_id = "/base_laser"
	
	#Loop
	rospy.loginfo("LIDAR Listener initialized")
	while not rospy.is_shutdown():
		try:
			raw_data =sock.recv(4096, socket.MSG_DONTWAIT)
			message.header.stamp=rospy.Time.now()
			if not raw_data:
				rospy.logwarn("No Raw Data")
			else:
				message.ranges=decode(raw_data)
				print message.ranges
				rospy.logdebug(message.ranges)
				laserpub.publish(message)
		except socket.error as ex:
			if (ex[0] != 11):
				rospy.logwarn( "Lidar Listener Socket Exception: ",ex)
			else:
				rospy.logdebug("Lidar Listener Timed Out");
		except Exception as ex:
			raise ex
	rospy.loginfo("Shutting down LIDAR listener...")
	sock.close()		
		
def decode(raw):
	lines = string.split(raw, "\n")
	data = [];
	for line in lines:
		for index in range(0,len(line)-1,2):
			value = struct.unpack('B',line[index:index+1])[0] - 0x30
			value <<= 6
			value &= ~0x003f
			value |= struct.unpack('B',line[index+1:index+2])[0] - 0x30
			data.append(value/1000.)
	return data

if __name__ == '__main__':
	try:
		lidar_listener()
	except rospy.ROSInterruptException: pass

import struct
import socket
import string
import threading, Queue
import cv, numpy, time
import math, signal
import random

#Data is a python list of unsigned ints (up to 16 bits per) w/ range in mm
#Mapper uses the following formula to convert datapoint index to angle (radians)
#	angle = (i * 0.0062832) + heading - 2.09 + 1.61
#Laser accuracy
#	Distance 20 mm ~ 4000 mm : +/-3 % of measurement
def main(q):
	print "Generating LIDAR Data..."
	for i in range(0,1000):
		generateFrame(q);

def generateFrame(q):
	max_range = 3300. #3.3 meters
	angular_res = 0.0062832 #in radians
	front_wall_y = 1000. #1 meters
	right_wall_x =  250. #.5 meter
	theta1 = math.radians(120) + math.acos(front_wall_y/max_range) #front wall comes into scanning range
	theta2 = math.radians(30) + math.atan(front_wall_y/right_wall_x) #corner position
	data = []
	for i in range(0,666):
		if (i * angular_res < theta1): #can see front wall
			value =  front_wall_y / math.cos(i * angular_res - math.radians(120))
			if (i * angular_res < theta2): #can see right wall
				value = right_wall_x / math.cos(i * angular_res - math.radians(30))
		else: #cannot see a wall
			value = max_range
		value = value * (0.97 + .06 * random.random()) #Add some random variation
		data.append(value);
	q.put(data);
	
if __name__ == "__main__":
	q = Queue.Queue()
	main(q)

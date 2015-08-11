import struct
import socket
import string
import threading, Queue
import cv, numpy, time
import math, signal
import sys

from lidar_listener import LidarListener

	
q = Queue.Queue()
listener = LidarListener(q)

b = True
x = 4000
y = 4000
heading = 0
start_index = 0

# Build the map, dummy
print "Building the map..."
m = cv.GetImage(cv.fromarray(255 * numpy.ones( (9000,9000,3), numpy.uint8 ) ))
cv.NamedWindow("map")

inner_mask = cv.GetImage(cv.fromarray(numpy.zeros( (600, 600), numpy.uint8 ) ))
outer_mask = cv.GetImage(cv.fromarray(numpy.zeros( (600, 600), numpy.uint8 ) ))


cv.SetImageROI(m, (3700, 3700, 600, 600))
cv.ShowImage("map", m)

gradient = (60, 60, 60)
grad2 = (120, 120, 120)

try:
	listener.daemon = True
	listener.start()
	while True:
		try:
			data = q.get(False)
			if data:
				cv.SetZero(inner_mask)
				cv.SetZero(outer_mask)
				inner_poly = []
				outer_poly = []
				cv.ResetImageROI(m)
				for data_point in range(len(data)):
					i = data_point + start_index
					ping = data[data_point] / 20.0
					valid_ping = True
					"""
					if (ping == 0.0):
						ping = 220
						valid_ping = False
					"""
					angle = (i * 0.0062832) + heading
					ping_x = int(math.floor(300 + ping*math.cos(angle)))
					ping_y = int(math.floor(300 - ping*math.sin(angle)))
					ping_xs = int(math.floor(300 + (ping*0.96)*math.cos(angle)))
					ping_ys = int(math.floor(300 - (ping*0.96)*math.sin(angle)))
					
					# Build the contours of the scanning masks
					inner_poly.append( (ping_x, ping_y) )
					cv.Circle(outer_mask, (ping_x, ping_y), 3, (255, 255, 255), -1)
					"""
					color = m[ping_x, ping_y][0] - 10*tau
					if (valid_ping):
						cv.Circle(m, (ping_x, ping_y), 5, (color, color, color), -1)
					cv.Line(m, (x, y), (ping_xs, ping_ys), (255, 255, 255), 5)
					cv.Line(mask, (x, y), (ping_x, ping_y), (1, 1, 1), 5)
					"""
				inner_poly.append( (x, y) )
				
				# Finally create the mask	
				cv.FillPoly(inner_mask, [inner_poly], 255)
				# Apply the masks to the map
				cv.SetImageROI(m, (3700, 3700, 600, 600))
				cv.AddS(m, grad2, m, inner_mask)
				cv.SubS(m, gradient, m, outer_mask)

				cv.ResetImageROI(m)				
				cv.Circle(m, (4000, 4000), 3, (255, 0, 0), -1)
				
				cv.SetImageROI(m, (3700, 3700, 600, 600))
				cv.ShowImage("map", m)
				cv.WaitKey(10)

		except KeyboardInterrupt:
			print "Caught the keyboard interrupt."
			listener.kill = True
			break
		except Exception as ex:
			pass
except KeyboardInterrupt:
	print "Caught a keyboard interrupt (out of loop)."
	listener.kill = True
	
print "At the end"
print "Queue size at end: ", q.qsize()
cv.DestroyWindow("map")
listener.kill = True













"""
listener.start()
failcount = 0

print "Starting the scan..."

# Quickly, do the shit!
while True:
	try:
		data = q.get(False)
		if data:
			print data
			for data_point in range(len(data)):
				i = data_point + start_index
				ping = data[data_point] / 10.0
				angle = (i * 0.0062832) + heading

				ping_x = int(math.floor(x + ping*math.cos(angle)))
				ping_y = int(math.floor(y - ping*math.sin(angle)))

				cv.Line(m, (x, y), (ping_x, ping_y), (255, 256, 256), 5)
				print "done computing points"
				
			cv.Circle(m, (4000, 4000), 3, (100, 100, 100), 4)

			cv.ShowImage("map", m)
			
		else:
			print "no usable data in the queue"

	except:
		pass


"""




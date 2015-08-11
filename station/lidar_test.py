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

# invent a destination
dest_x = 4000
dest_y = 3980
dest_conv = 10
d_x = 4000
d_y = 4000

# Build the map, dummy
print "Building the map..."
m = cv.GetImage(cv.fromarray(255 * numpy.ones( (9000,9000,3), numpy.uint8 ) ))
cv.NamedWindow("map")

inner_mask = cv.GetImage(cv.fromarray(numpy.zeros( (600, 600), numpy.uint8 ) ))
outer_mask = cv.GetImage(cv.fromarray(numpy.zeros( (600, 600), numpy.uint8 ) ))


cv.SetImageROI(m, (3700, 3700, 600, 600))
cv.ShowImage("map", m)

toward_white = (60, 60, 60)
toward_black = (120, 120, 120)

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
					if ping == 0:
						ping = 220
						valid_ping = False
					
					angle = (i * 0.0062832) + heading - 2.09 + 1.61
					ping_x = int(math.floor(300 + ping*math.cos(angle)))
					ping_y = int(math.floor(300 - ping*math.sin(angle)))
					ping_xs = int(math.floor(300 + (ping*0.96)*math.cos(angle)))
					ping_ys = int(math.floor(300 - (ping*0.96)*math.sin(angle)))
					
					# Build the contours of the scanning masks
					inner_poly.append( (ping_x, ping_y) )
					if valid_ping == True:
						cv.Circle(outer_mask, (ping_x, ping_y), 3, (255, 255, 255), -1)
					
				inner_poly.append( (x, y) )
				
				# Finally create the mask	
				cv.FillPoly(inner_mask, [inner_poly], 255)
				
				# Apply the masks to the map
				cv.SetImageROI(m, (3700, 3700, 600, 600))
				cv.AddS(m, toward_white, m, inner_mask)
				cv.SubS(m, toward_black, m, outer_mask)

				cv.ResetImageROI(m)				
				cv.Circle(m, (4000, 4000), 3, (255, 0, 0), -1)
				
				
				# Draw poo
				#cv.ResetImageROI(m)
				#cv.Circle(m, (d_x, d_y), 3, (0, 0, 255), -1)
				#cv.Circle(m, (dest_x, dest_y), dest_conv, (0, 255, 255), 1)
				#cv.Line(m, (d_x, d_y), (dest_x, dest_y), (255, 255, 0))
				
				# Is the drone outside the destination?
				if ( (x-dest_x)^2 + (y-dest_y)^2 > dest_conv^2):
					
					# Compute the direction of travel
					trav_angle = cv.FastArctan( (dest_x-x), (dest_y-y) )
					trav_dist = cv.Sqrt( (dest_x-x)^2 + (dest_y-y)^2 )
					trav_x = trav_dist * math.cos(trav_angle)
					trav_y = trav_dist * math.sin(trav_angle)
					
					# Detect if someone has walked inside the bounding box
					drone_radius = 10
					cv.SetImageROI(m, (dest_x-drone_radius, dest_y-drone_radius, 2*drone_radius, 2*drone_radius))
					result = cv.CreateMat(2*drone_radius, 2*drone_radius, cv.CV_8UC3)
					cv.AndS(m, 255, result)
					
					conflicts = 0
					for i in range(2*drone_radius):
						for j in range(2*drone_radius):
							if (result[i,j][0] == 0):
								conflicts = conflicts + 1
								
					thickness = 1
					if (conflicts > 0.0):
						print "GTFO!"
						thickness = -1
					
					cv.ResetImageROI(m)
					cv.Rectangle(m, (dest_x-drone_radius, dest_y-drone_radius), (dest_x+drone_radius, dest_y+drone_radius), (255, 255, 0), thickness)

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




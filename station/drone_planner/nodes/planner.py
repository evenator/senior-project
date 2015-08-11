#!/usr/bin/env python

# ROS related imports
import roslib; roslib.load_manifest('drone_planner')
import rospy
from nav_msgs.msg import OccupancyGrid
import cv, numpy, math

# Planner related imports
import os,sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))
from mapper import Mapper
from waypoint import *
from planner_network import PlannerNetworkManager
from planner_wallcrawl import WallCrawl
from Queue import Queue
from pso_network import PlannerSender
from planner_exporter import PlannerExporter

# Planner constants
(AUTO, WAYPOINT, MANUAL) = range(3)
REFLEXIVE_HALT_RADIUS = 8
test = True

# Create global manager objects
mapperManager = Mapper()
uiManager = None
if (not test):
	uiManager = PlannerNetworkManager()
#psoManager = PlannerSender()
goalManager = WallCrawl()
exportManager = PlannerExporter(mapperManager.mapPointer())

def mapCallback(data):

	global mapperManager
	
	while (mapperManager.lock == True):
		pass
		
	mapperManager.lock = True

	d = data.data
	i = data.info
	
	# mapper space
	x = 2000
	y = 2000
	face = 450
	m = mapperManager.mapPointer()
	
	begin_x = x - (face/2)	
	begin_y = y - (face/2)
	for row in range(face):
		for col in range(face):
			seek = (begin_x*i.width) + (row*i.width) + col + begin_y
			if (d[seek] > -1):
				datum = (100-d[seek])*2.55
				m[begin_x + row, begin_y + col] = (datum, datum, datum, datum)
	
	mapperManager.lock = False
	"""
	data_array = numpy.reshape(numpy.array(d), (i.height, i.width))
	pre_image = numpy.zeros( (i.height, i.width, 4), numpy.uint8 )
	for i in range(4):
		pre_image[:,:,i] = data_array
	mapperManager.updateFromROS(cv.GetImage(cv.fromarray(pre_image)))
	"""
			
	"""
	x_offset = abs(i.origin.position.x / i.resolution)
	y_offset = abs(i.origin.position.y / i.resolution)
	
	begin_x = int(x_offset + (x / i.resolution) - (image_face/2))
	begin_y = int(y_offset + (y / i.resolution) - (image_face/2))
	
	for row in range(image_face):
		seek = ((begin_x*width) + (row*width) + begin_y)
		row_data = numpy.array(d[seek:seek+image_face])
		row_data = numpy.clip(row_data, 0, 100)
		row_data = (100-row_data)*2.55
		for i in range(4):
			a[row,:,i] = row_data
	global m
	m = cv.GetImage(cv.fromarray(a))
	"""
	
def planner():
	try:
		# Subscribe the the ROS SLAM output
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("map", OccupancyGrid, mapCallback)
		
		# Instantiate manager objects
		global mapperManager
		if (not test):
			global uiManager
		#global psoManager
		global exportManager
				
		# Planner parameters
		roi_face = mapperManager.roi_size
		screen_image = cv.GetImage(cv.fromarray(numpy.zeros((roi_face, roi_face, 4), numpy.uint8)))
		flight_mode = AUTO
		waypointQ = []
		if test:
			cv.NamedWindow("map")
				
		# Start the LIDAR thread
		mapperManager.start()
		
		# Build the reflexive halt mask
		reflexive_halt_mask = cv.GetImage(cv.fromarray(numpy.zeros((24, 24), numpy.uint8)))
		cv.Circle(reflexive_halt_mask, (12, 12), REFLEXIVE_HALT_RADIUS, 255, -1)
				
		while True:			
			# Need to replace this with navdata, currently the center of the map
			x = 2000
			y = 2000
			heading = 0
			roi_x = x - (roi_face/2)
			roi_y = y - (roi_face/2)
			"""
			uicommand = uiManager.getUICommand()
			if uicommand:
				tokens = str.split(uicommand)
				if (tokens[0] == "click"):
					print "a click has gone through"
				if (tokens[0] == "mode"):
					if (tokens[1] == '0'):
						print "Entering autonomous mode"
						flight_mode = AUTO
					elif (tokens[1] == '1'):
						print "Entering manual waypoint command mode."
						flight_mode = WAYPOINT
					elif (tokens[1] == '2'):
						print "Entering manual controller command mode."
						flight_mode = MANUAL
				if (tokens[0] == "export"):
					exportManager.run()
			"""
			# Get the map scan
			"""
			mapperManager.scan()
			"""
			m = mapperManager.mapPointer()
			
			
			# Create the screen buffer 
			cv.SetImageROI(m, (roi_x, roi_y, roi_face, roi_face))
			cv.Copy(m, screen_image)
			cv.ResetImageROI(m)
			
			# Check for a reflexive halt condition
			halt_roi = (x-12, y-12, 24, 24)
			halt = mapperManager.obstaclePresentMask(reflexive_halt_mask, halt_roi)
			cv.Circle(screen_image, (roi_face/2, roi_face/2), REFLEXIVE_HALT_RADIUS, (0, 0, 255, 0), 1)
			if (halt):
				print "REFLEXIVE HALT!"
			
			
			# Prevent the LIDAR from ghosting itself
			cv.Circle(m, (x, y), 10, (255, 255, 255, 255), -1)
			
			# Perform the appropriate tfask
			if (flight_mode == AUTO):
				new_waypoints = []
				if (len(waypointQ) < 10):
					new_waypoints = goalManager.computeNewWaypoints(mapperManager, waypointQ, None, 4, screen_image)
				# this will have to change when we figure out how to do waypoint arrival
				waypointQ = new_waypoints
			elif (flight_mode == WAYPOINT):
				pass
			elif (flight_mode == MANUAL):
				pass				
			
			
			# Commit graphics to the screen buffer
			old_x = 225
			old_y = 225
			for w in waypointQ:
				plot_x = w[0] - x + (roi_face/2)
				plot_y = w[1] - y + (roi_face/2)
				cv.Line(screen_image, (old_x, old_y), (plot_x, plot_y), (0, 0, 255, 0), 1)
				old_x = plot_x
				old_y = plot_y
				#cv.Circle(screen_image, (plot_x, plot_y), 15, (0, 0, 255, 0), 1)
				#cv.Circle(screen_image, (plot_x, plot_y), 6, (0, 0, 255, 0), 1)
				cv.Circle(screen_image, (plot_x, plot_y), 2, (0, 0, 255, 0), -1)
			angle = heading + 1.61
			heading_x = int(math.floor(roi_face/2 + 12*math.cos(angle)))
			heading_y = int(math.floor(roi_face/2 - 12*math.sin(angle)))
			cv.Circle(screen_image, (roi_face/2, roi_face/2), 6, (255, 0, 0, 0), -1)
			cv.Line(screen_image, (roi_face/2, roi_face/2), (heading_x, heading_y), (0, 255, 0), 1)
			
			if test:
				cv.ShowImage("map", screen_image)
				cv.WaitKey(10)
			
			
			# Transmit the results to the appropriate places
			if (not test):
				uiManager.sendLidarString(screen_image.tostring())
	
	except KeyboardInterrupt:
		print "Caught the keyboard interrupt!"
	except Exception as ex:
		print "Caught an unhandled exception:"
		print ex
	
	if (mapperManager):
		mapperManager.shutdown()
	if (uiManager):
		uiManager.shutdown()





if __name__ == '__main__':
	try:
		planner()
	except rospy.ROSInterruptException as ex:
		print ex




"""
def planner():
    laserpub = rospy.Publisher('scan', LaserScan)
    rospy.init_node('planner')
    while not rospy.is_shutdown():
        rospy.loginfo(str)
        laserpub.publish(LaserScan(None))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        planner()
    except rospy.ROSInterruptException: pass
"""

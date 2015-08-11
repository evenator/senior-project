# Drone Project Planner
# This module wraps the mapper object and performs the tasks of path planning and goal selecting

import numpy, cv, math, socket, os

from mapper import Mapper
from waypoint import *
from planner_network import PlannerNetworkManager
from planner_wallcrawl import WallCrawl
from Queue import Queue
from pso_network import PlannerSender
from planner_exporter import PlannerExporter

(AUTO, WAYPOINT, MANUAL) = range(3)
REFLEXIVE_HALT_RADIUS = 8
test = False

def main():
	try:
		mapperManager = Mapper()
		if (not test):
			uiManager = PlannerNetworkManager()
		#psoManager = PlannerSender()
		goalManager = WallCrawl()
		exportManager = PlannerExporter(mapperManager.mapPointer())
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
			x = 2000
			y = 2000
			heading = 0
			roi_x = x - (roi_face/2)
			roi_y = y - (roi_face/2)
			
			if (not test):
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
			
			mapperManager.scan()
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
	if (not test and uiManager):
		uiManager.shutdown()
		



main()











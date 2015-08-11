# Class which implements the wall crawling method for path planning
from planner_network import PlannerNetworkManager
import cv, numpy, math

class WallCrawl:
	
	def __init__(self):
		# These are distances defined in pixels
		self.__majord = 40
		self.__minord = 12
		self.__maxReach = 20
		
	def computeNewWaypoints(self, mapperManager, waypointQ, state, steps, screen_image):
		while (mapperManager.lock == True):
			pass
	
		mapperManager.lock = True
		# Use the state to locate the centroid of the region of interest
		x = 2000
		y = 2000
		heading = 0
		# Compute the drone's safety region in the global map
		mask_roi = ( (x-(self.__majord*1.414), y-(self.__majord*1.414)), (2*self.__majord, 2*self.__majord) )
		
		# Determine the numbers of steps to compute in this iteration
		if (steps > self.__maxReach):
			steps = self.__maxReach
		x_offset = x
		y_offset = y
		heading_offset = heading
		good_steps = 0
		new_waypoints = []
		while good_steps < steps:
			inner_ring_safe = False
			outer_wall_present = False
			
			# Create dummy waypoints and masks for object detection
			w = ( (x_offset, y_offset), (x_offset, y_offset) )
			outer_detection_mask = cv.GetImage(cv.fromarray( numpy.zeros( (self.__majord, self.__majord), numpy.uint8 )))
			cv.Circle(outer_detection_mask, (self.__majord/2, self.__majord/2), self.__majord/2, 255, -1)
			# Loop over almost an entire circle (60 in range, divided by 10 inside, 6 radians overall)
			for i in range(60):
				
				# Compute the location of the candidate step
				project_angle = (i/10.0) + heading + 1.61 - 1.57 + heading_offset
				project_x = int(math.floor(x_offset - self.__minord*math.cos(project_angle)))
				project_y = int(math.floor(y_offset - self.__minord*math.sin(project_angle)))
				w = (w[0], (project_x, project_y))
				
				#cv.Line(screen_image, (w[0][0]-3775,w[0][1]-3775), (w[1][0]-3775,w[1][1]-3775), (4*i, 4*i, 0, 0), 2)
				
				# Check if the straight line path is safe to travel
				inner_ring_safe = mapperManager.safeLine( ((x_offset, y_offset),(project_x,project_y)), self.__minord)
				
				# Check if the outer ring still contains a wall
				outer_roi = (project_x-(self.__majord/2), project_y-(self.__majord/2), self.__majord, self.__majord)
				outer_wall_present = mapperManager.obstaclePresentMask(outer_detection_mask, outer_roi)
				
				"""
				if (outer_wall_present):
					pass
					#cv.Circle(screen_image, (project_x-3775, project_y-3775), self.__majord/2, (255, 255, 0, 0))
				if (inner_ring_safe):
					pass
					#cv.Line(screen_image, (w[0][0]-3775,w[0][1]-3775), (w[1][0]-3775,w[1][1]-3775), (4*i, 4*i, 0, 0), 2)
				"""
				if (inner_ring_safe and outer_wall_present):
					cv.Circle(screen_image, (project_x-1775, project_y-1775), self.__majord/2, (200, 200, 0, 0), 1)
					cv.Circle(screen_image, (project_x-1775, project_y-1775), self.__minord/2, (200, 0, 200, 0), 1)
					new_waypoints.append( (project_x, project_y, project_angle) )
					x_offset = project_x
					y_offset = project_y
					break
				else:
					if (i == 59):
						print "WALL CRAWLER HAS FAILED TO FIND AN APPROPRIATE STEP ON #", good_steps
						good_steps = steps
			good_steps = good_steps + 1
		mapperManager.lock = False
		return new_waypoints
		
		
		
		
		

# Drone Mapper

import numpy, cv
import threading, Queue, math
import lidar_generator

from lidar_listener import LidarListener

class Mapper:

	def __init__(self, simdata=False):
		self.roi_size = 450
		self.darken_gradient = (30, 30, 30, 30)
		self.lighten_gradient = (70, 70, 70, 70)
		self.mm_per_pixel = 15.0
		self.lock = False
		
		self.__q = Queue.Queue()
		self.simdata = simdata;
		if not simdata:
			self.__listener = LidarListener(self.__q)
		else:
			lidar_generator.main(self.__q)
		self.__m = cv.GetImage(cv.fromarray(127 * numpy.ones( (4000,4000,4), numpy.uint8 )))
		self.__innermask = cv.GetImage(cv.fromarray(numpy.zeros( (self.roi_size, self.roi_size), numpy.uint8 )))
		self.__outermask = cv.GetImage(cv.fromarray(numpy.zeros( (self.roi_size, self.roi_size), numpy.uint8 )))
		
	def start(self):
		if not self.simdata:
			try:
				self.__listener.daemon = True
				self.__listener.start()
			except Exception as ex:
				raise ex
			
	def shutdown(self):
		if not self.simdata:
			try:
				self.__listener.kill = True
			except Exception as ex:
				raise ex
		
	def obstaclePresent(self, abs_center, radius):
		roi = (abs_center[0]-radius, abs_center[1]-radius, 2*radius, 2*radius)	
		cv.SetImageROI(self.__m, roi)
		result = cv.CreateMat(2*radius, 2*radius, cv.CV_8UC4)
		cv.AndS(self.__m, (255, 255, 255), result)
		result_sum = ((255 * 4 * radius * radius) - cv.Sum(result)[0])	
		"""
		conflicts = 0
		for i in range(2*radius):
			for j in range(2*radius):
				if (result[i,j][0] < 255):
					conflicts = conflicts + 1
		result_sum = conflicts
		"""
		cv.ResetImageROI(self.__m)		
		return (result_sum > 0.0)
		
		
	# Could be improved, not focused on it right now
	def obstaclePresentMask(self, mask, mask_roi):
		cv.SetImageROI(self.__m, mask_roi)
		result = cv.CreateMat(mask.height, mask.width, cv.CV_8UC4)
		
		cv.AndS(self.__m, (255, 255, 255), result, mask)
		conflicts = 0
		for i in range(mask.height):
			for j in range(mask.width):
				if (result[i,j][0] < 255 and mask[i,j] != 0):
					conflicts = conflicts + 1
					
		#result_sum = ((255*mask.height*mask.width) - cv.Sum(result)[0])
		
		cv.ResetImageROI(self.__m)
		return (conflicts > 0)
		
	def safeLine(self, waypoint, thickness):	
		cv.ResetImageROI(self.__m)
		conflicts = 0
		angle = math.radians(cv.FastArctan(waypoint[0][1]-waypoint[1][1], waypoint[0][0]-waypoint[1][0]))
		for i in range(cv.Floor( cv.Sqrt( math.pow(waypoint[0][0]-waypoint[1][0], 2) + math.pow(waypoint[0][1]-waypoint[1][1], 2) ) )):
			check_x = cv.Floor(waypoint[0][0] - i*math.cos(angle))
			check_y = cv.Floor(waypoint[0][1] - i*math.sin(angle))
			if self.obstaclePresent( (check_x, check_y), thickness/2):
				conflicts = conflicts + 1
		return (conflicts == 0)
			
	def mapImage(self):
		mapCopy = cv.GetImage(cv.fromarray( numpy.zeros ( (self.roi_size, self.roi_size), numpy.uint8 )))
		cv.SetImageROI(self.__m, (x-(self.roi_size/2), y-(self.roi_size/2), self.roi_size, self.roi_size))
		cv.Copy(self.__m, mapCopy)
		cv.ResetImageROI(self.__m)
		return mapCopy
		
	def mapCopy(self, roi):
		mapCopy = cv.GetImage(cv.fromarray( numpy.zeros ( (roi[2], roi[3]), numpy.uint8 )))
		cv.SetImageROI(self.__m, roi)
		cv.Copy(self.__m, mapCopy)
		cv.ResetImageROI(self.__m)
		return mapCopy
	
	def mapPointer(self):
		return self.__m
	
	def updateFromROS(self, newMap):
		self.__m = newMap
		cv.ResetImageROI(self.__m)
	
	def scan(self):
		x = 2000
		y = 2000
		heading = 0
		
		try:
			data = self.__q.get(False)
			if data:
				cv.ResetImageROI(self.__m)
				cv.SetZero(self.__innermask)
				cv.SetZero(self.__outermask)
				inner_poly = []
				
				for i in range(len(data)):
					
					ping = data[i] / self.mm_per_pixel

					# This needs to be replaced by an error code
					valid_ping = True
					if (ping == 0):
						ping = 220
						valid_ping = False
					
					# This needs to be replaced with a navdata structure
					angle = (i * 0.0062832) + heading - 2.09 + 1.61
					ping_x = int(math.floor((self.roi_size/2) + ping*math.cos(angle)))
					ping_y = int(math.floor((self.roi_size/2) - ping*math.sin(angle)))
										
					# Build the contours of the scanning masks
					inner_poly.append( (ping_x, ping_y) )
					if valid_ping == True:
						cv.Circle(self.__outermask, (ping_x, ping_y), 3, 255, -1)
					
				# Append the first ping to close the polygon's contour
				inner_poly.append( inner_poly[0] )
				
				
				# Finally create the inner scan mask
				cv.FillPoly(self.__innermask, [inner_poly], 255)
				
				
				# Apply the masks to the map
				cv.SetImageROI(self.__m, (x-(self.roi_size/2), y-(self.roi_size/2), self.roi_size, self.roi_size))
				cv.SubS(self.__m, self.darken_gradient, self.__m, self.__outermask)
				cv.AddS(self.__m, self.lighten_gradient, self.__m, self.__innermask)
				cv.ResetImageROI(self.__m)
				
		except Exception as ex:
			cv.ResetImageROI(self.__m)
			pass
			

import csv
import time
import cv
import numpy
class Logger:
	def __init__(self):
		self.f = open('logs/'+time.strftime('%Y-%m-%d_%H%M')+'.csv', 'wb')
		self.writer = csv.writer(self.f)
		#self.writer.writerow(('x','dx','y','dy','z','dz','psi','dpsi')) 
	
	def log(self, data):
		data = numpy.asarray(data)
		data = numpy.transpose(data)
		data = data.tolist()
		self.writer.writerow(data[0])

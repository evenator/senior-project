import time
import os

class Logger:
	def __init__(self):
		if(os.path.exists('/media/FreeAgent GoFlex Drive/')):
			self.f = open('/media/FreeAgent GoFlex Drive/logs/'+time.strftime('%Y-%m-%d_%H%M')+'.txt', 'wb')
		else:
			self.f = open('logs/'+time.strftime('%Y-%m-%d_%H%M')+'.txt', 'wb')

	def log(self, data):
		data = time.strftime('%Y-%m-%d_%T') + ": " + data + '\n'
		self.f.write(data)
		#print data

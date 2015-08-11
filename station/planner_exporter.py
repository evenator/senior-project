from threading import Thread
import cv, time, os

class PlannerExporter(Thread):
	def __init__(self, mapPointer):
		Thread.__init__(self)
		self.__m = mapPointer
		self.running = False
		
	def run(self):
		self.running = True
		print "Exporting the entire map..."
		cv.ResetImageROI(self.__m)
		if(os.path.exists('/media/FreeAgent GoFlex Drive/')):
			mapfile = '/media/FreeAgent GoFlex Drive/maps/'+time.strftime('%Y-%m-%d_%H%M')+'.png'
		else:
			mapfile = 'maps/'+ time.strftime('%Y-%m-%d_%T') + '.png'	
		cv.SaveImage(mapfile, self.__m)
		print "Done! Saved...", mapfile
		self.running = False

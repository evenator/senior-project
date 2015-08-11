# Classes for each waypoint

class IdleWaypoint:
	def __init__(self, center):
		self.center = center

class TurnWaypoint:
	def __init__(self, center, angle):
		self.center = center
		self.angle = angle
		
class StraightLine:
	def __init__(self, source, dest):
		self.source = source
		self.dest = dest
		
	



import math

def world2drone(x, y, heading):
	drone_x = x * math.cos(heading) - y * math.sin(heading)
	drone_y = x * math.sin(heading) + y * math.cos(heading)
	return (drone_x, drone_y)

print world2drone(1, 0, math.radians(90))
print world2drone(0, 1, math.radians(90))
